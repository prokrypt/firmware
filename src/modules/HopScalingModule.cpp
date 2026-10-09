#include "HopScalingModule.h"
#include "SafeFile.h"
#include "meshUtils.h"

#if HAS_VARIABLE_HOPS

#include "FSCommon.h"
#include "NodeDB.h"
#include "SPILock.h"
#include "UptimeClock.h"
#include "airtime.h"
#include "concurrency/LockGuard.h"
#include "gps/RTC.h"
#include "mesh-pb-constants.h"
#include <algorithm>
#include <cmath>
#include <cstring>

namespace
{
// Module scheduling
constexpr uint32_t INITIAL_DELAY_MS = 30 * 1000UL;    // Startup grace period before first run
constexpr uint32_t RUN_INTERVAL_MS = 5 * 60 * 1000UL; // Emit micro-summary every 5 minutes
// RUNS_PER_HOUR is a public class constant in HopScalingModule.h

// Persistence: written only on a user shutdown or reboot, never on a timer.
// 'HST3' adds the save time; an older file (no save time, so no way to age it) fails the magic check.
constexpr uint32_t HISTOGRAM_STATE_MAGIC = 0x48535433; // 'HST3'
constexpr uint8_t HISTOGRAM_STATE_VERSION = 1;
constexpr const char *HISTOGRAM_STATE_FILE = "/prefs/hopScalingState.bin";
constexpr uint8_t SEEN_WINDOW_HOURS = 13; // width of Record::seenHoursAgo

} // namespace

#pragma pack(push, 1)
struct HopScalingModule::PersistedHistogram {
    uint32_t magic;
    uint8_t version;
    uint8_t samplingDenominator;
    uint8_t filteringDenominator;
    uint8_t filterDenomHoldRollsRemaining; // rollHour() calls remaining in the hold; 0 when expired/not active
    uint16_t hashSeed;
    uint32_t savedAtEpoch;                      // wall clock at save; the snapshot ages against it
    Record entries[HopScalingModule::CAPACITY]; // full 512-byte array; count derived on load
};
#pragma pack(pop)

HopScalingModule *hopScalingModule;

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

HopScalingModule::HopScalingModule() : concurrency::OSThread("HopScaling")
{
    clear();
    loadFromDisk();
    setIntervalFromNow(INITIAL_DELAY_MS);
}

HopScalingModule::~HopScalingModule() = default;

#ifndef PIO_UNIT_TESTING
uint32_t HopScalingModule::wallClock()
{
    return getValidTime(RTCQualityDevice);
}
#else
void HopScalingModule::stageSnapshotForTest(const HopScalingModule &from, uint32_t savedAtEpoch)
{
    pendingSnapshot.reset(new PersistedHistogram{});
    pendingSnapshot->samplingDenominator = from.samplingDenominator;
    pendingSnapshot->filteringDenominator = from.filteringDenominator;
    pendingSnapshot->filterDenomHoldRollsRemaining = from.filteringDenomHoldRollsRemaining;
    pendingSnapshot->hashSeed = from.hashSeed;
    pendingSnapshot->savedAtEpoch = savedAtEpoch;
    memcpy(pendingSnapshot->entries, from.entries, sizeof(from.entries));
    hashSeed = from.hashSeed;
}
#endif

void HopScalingModule::clear()
{
    memset(entries, 0, sizeof(entries));
    this->count = 0;
    samplingDenominator = DENOM_MIN;
    filteringDenominator = DENOM_MIN;
    filteringDenomHoldRollsRemaining = 0;
    lastPerHopCounts = {};
    lastSuggestedHop = MAX_HOP;
    lastPoliteNumer = POLITENESS_DEFAULT;
    lastTrendStats = {};
    memset(denominatorHistory, DENOM_MIN, sizeof(denominatorHistory));
    pendingSnapshot.reset();
#ifndef PIO_UNIT_TESTING
    hashSeed = static_cast<uint16_t>(random());
#else
    hashSeed = 0; // deterministic in unit tests
#endif
}

// ---------------------------------------------------------------------------
// Persistence
// ---------------------------------------------------------------------------

void HopScalingModule::saveToDisk()
{
#ifdef FSCom
    // Fold in a snapshot still waiting for a clock, so a shutdown soon after boot doesn't lose it.
    adoptSnapshotIfReady();

    const uint32_t now = wallClock();
    if (now == 0) {
        LOG_INFO("[HOPSCALE] No clock, not saving: the snapshot could never be aged");
        return;
    }
    if (count == 0)
        return;

    FSCom.mkdir("/prefs");
    PersistedHistogram state{};
    state.magic = HISTOGRAM_STATE_MAGIC;
    state.version = HISTOGRAM_STATE_VERSION;
    state.samplingDenominator = samplingDenominator;
    state.filteringDenominator = filteringDenominator;
    state.filterDenomHoldRollsRemaining = filteringDenomHoldRollsRemaining;
    state.hashSeed = hashSeed;
    state.savedAtEpoch = now;
    // Save all CAPACITY slots; count is reconstructed on load by scanning seenHoursAgo.
    memcpy(state.entries, entries, sizeof(state.entries));
    auto file = SafeFile(HISTOGRAM_STATE_FILE, true);
    const size_t written = file.write(reinterpret_cast<const uint8_t *>(&state), sizeof(state));
    if (!file.close() || written != sizeof(state))
        LOG_WARN("[HOPSCALE] State save failed");
#endif
}

void HopScalingModule::loadFromDisk()
{
#ifdef FSCom
    auto state = std::unique_ptr<PersistedHistogram>(new PersistedHistogram{});
    {
        concurrency::LockGuard g(spiLock);
        auto file = FSCom.open(HISTOGRAM_STATE_FILE, FILE_O_READ);
        if (!file)
            return;
        const bool readOk = (file.read(reinterpret_cast<uint8_t *>(state.get()), sizeof(*state)) == sizeof(*state));
        file.close();
        if (!readOk)
            return;
    }
    // Validate magic, version, denom range, denom power-of-two invariant, and hold counter.
    if (state->magic != HISTOGRAM_STATE_MAGIC || state->version != HISTOGRAM_STATE_VERSION ||
        state->samplingDenominator < DENOM_MIN || state->samplingDenominator > DENOM_MAX ||
        state->filteringDenominator < state->samplingDenominator || state->filteringDenominator > DENOM_MAX ||
        !is_pow_of_2(state->samplingDenominator) || !is_pow_of_2(state->filteringDenominator) ||
        state->filterDenomHoldRollsRemaining > FILTER_DENOM_HOLD_ROLLS || state->savedAtEpoch == 0) {
        return;
    }
    // The snapshot is held until the clock says how old it is. Take its hash seed now so nodes sampled
    // in the meantime hash the same way and merge with it.
    hashSeed = state->hashSeed;
    pendingSnapshot = std::move(state);
#endif
}

void HopScalingModule::discardSnapshot(const char *why)
{
    LOG_INFO("[HOPSCALE] Saved histogram discarded: %s", why);
    pendingSnapshot.reset();
}

void HopScalingModule::adoptSnapshotIfReady()
{
    if (!pendingSnapshot)
        return;

    const uint32_t now = wallClock();
    if (now == 0) {
        // Without a clock its age is unknown; once we've been up a full window it would be expired anyway.
        if (Time::getUptimeSecs() >= SEEN_WINDOW_HOURS * 3600UL)
            discardSnapshot("no clock within the 13 h window");
        return;
    }
    const PersistedHistogram &snap = *pendingSnapshot;
    if (snap.savedAtEpoch > now) {
        discardSnapshot("saved in the future");
        return;
    }
    const uint32_t ageHours = (now - snap.savedAtEpoch + 1800) / 3600;
    if (ageHours >= SEEN_WINDOW_HOURS) {
        discardSnapshot("older than 13 h");
        return;
    }

    // Age the snapshot's denominator hold the way ageHours of rollHour() calls would have.
    uint8_t snapFilt = snap.filteringDenominator;
    uint8_t snapHold = snap.filterDenomHoldRollsRemaining;
    for (uint32_t h = 0; h < ageHours && snapFilt > snap.samplingDenominator; h++) {
        if (snapHold > 0)
            snapHold--;
        if (snapHold == 0) {
            const uint8_t stepped = static_cast<uint8_t>(snapFilt / 2u);
            snapFilt = (stepped > snap.samplingDenominator) ? stepped : snap.samplingDenominator;
        }
    }
    samplingDenominator = std::max(samplingDenominator, snap.samplingDenominator);
    filteringDenominator = std::max({filteringDenominator, snapFilt, samplingDenominator});
    filteringDenomHoldRollsRemaining = std::max(filteringDenomHoldRollsRemaining, snapHold);

    // Merge: live entries first (their hop counts are fresher), then the aged snapshot, OR-ing seen bits on a match.
    Record merged[CAPACITY] = {};
    uint8_t n = 0;
    for (uint8_t i = 0; i < count; i++) {
        if (passesFilter(entries[i].nodeHash, samplingDenominator))
            merged[n++] = entries[i];
    }
    for (uint8_t i = 0; i < CAPACITY; i++) {
        const uint32_t seen = (static_cast<uint32_t>(snap.entries[i].seenHoursAgo) << ageHours) & 0x1FFFu;
        if (seen == 0u || !passesFilter(snap.entries[i].nodeHash, samplingDenominator))
            continue;
        bool matched = false;
        for (uint8_t j = 0; j < n; j++) {
            if (merged[j].nodeHash == snap.entries[i].nodeHash) {
                merged[j].seenHoursAgo |= seen;
                matched = true;
                break;
            }
        }
        if (!matched && n < CAPACITY) {
            merged[n] = snap.entries[i];
            merged[n].seenHoursAgo = seen;
            n++;
        }
    }
    memcpy(entries, merged, sizeof(entries));
    this->count = n;
    // denominatorHistory can't be recovered; raise every slot to the filter so post-boot hourly
    // estimates use a safe (slightly conservative) multiplier.
    for (uint8_t h = 0; h < SEEN_WINDOW_HOURS; h++)
        denominatorHistory[h] = std::max(denominatorHistory[h], filteringDenominator);
    if (getFillPercentage() >= FILL_HIGH_PCT)
        trimIfNeeded();

    LOG_INFO("[HOPSCALE] Saved histogram adopted: %u h old, %u entries", static_cast<unsigned>(ageHours),
             static_cast<unsigned>(count));
    pendingSnapshot.reset();
}

// ---------------------------------------------------------------------------
// Core API
// ---------------------------------------------------------------------------

void HopScalingModule::samplePacketForHistogram(uint32_t nodeId, uint8_t hopCount)
{
    const uint16_t hash = hashNodeId(nodeId);

    if (!passesFilter(hash, samplingDenominator))
        return;

    hopCount = std::min(hopCount, MAX_HOP);

    // Update an existing entry
    Record *entry = nullptr;
    for (uint8_t i = 0; i < count; i++) {
        if (entries[i].nodeHash == hash) {
            entry = &entries[i];
            break;
        }
    }
    if (entry) {
        entry->hops_away = hopCount;
        markCurrentHour(*entry);
        return;
    }

    // New node: trim if necessary before allocating a slot
    if (getFillPercentage() >= FILL_HIGH_PCT) {
        trimIfNeeded();
    }

    if (count < CAPACITY) {
        entries[count].nodeHash = hash;
        entries[count].hops_away = hopCount;
        entries[count].seenHoursAgo = 1u; // mark current hour
        this->count++;
    } else {
        LOG_WARN("[HOPSCALE] Histogram full, node dropped");
    }
}

void HopScalingModule::rollHour()
{
    // Advance denominatorHistory before the tally so each slot h holds the filteringDenominator
    // that was active when seenHoursAgo bit h was set.  hourlyRaw[h] is then gated per-slot by
    // denominatorHistory[h], giving a correct population estimate for each historical hour even
    // when filteringDenominator changes between rolls.  Scale-up backfills the entire array so
    // the invariant holds retroactively (see trimIfNeeded()).
    for (uint8_t h = 12; h > 0; h--)
        denominatorHistory[h] = denominatorHistory[h - 1];
    denominatorHistory[0] = filteringDenominator;

    // 1. Tally per-hop counts and per-slot hourly activity in one pass.
    //    hourlyRaw[h]: gated per-slot by denominatorHistory[h] so the raw count and its
    //      multiplier are always consistent, even across filteringDenominator transitions.
    //    counts.*: gated uniformly by the current filteringDenominator for a consistent
    //      population estimate used by the hop-walk recommendation (step 2).
    PerHopCounts counts{};
    uint16_t hourlyRaw[13] = {};
    uint16_t trendNewThisHour = 0;
    uint16_t trendReturning = 0;
    uint16_t trendLapsed = 0;
    uint16_t trendOlderThan4h = 0;
    uint16_t trendAgingOut = 0;
    for (uint8_t i = 0; i < count; i++) {
        const uint16_t hash = entries[i].nodeHash;
        const uint32_t seen = entries[i].seenHoursAgo;

        // Per-slot hourly activity: gate each slot by its own denominator.
        for (uint8_t h = 0; h < 13; h++) {
            if ((seen & (1u << h)) && passesFilter(hash, denominatorHistory[h]))
                hourlyRaw[h]++;
        }

        // Hop counts and trend stats: uniform current-denominator gate.
        if (!passesFilter(hash, filteringDenominator))
            continue;

        if (seenInLast13h(entries[i])) {
            counts.perHop[entries[i].hops_away]++;
            counts.total++;
        }
        const bool heardThisHour = (seen & 1u) != 0u;
        const bool heardLastHour = (seen & 2u) != 0u;
        const bool hasOlderHistory = (seen >> 1u) != 0u;
        const bool recentlySilent = (seen & 0xFu) == 0u;
        if (heardThisHour && !hasOlderHistory)
            trendNewThisHour++;
        else if (heardThisHour && hasOlderHistory)
            trendReturning++;
        if (!heardThisHour && heardLastHour)
            trendLapsed++;
        if (recentlySilent && (seen & 0x1FF0u) != 0u)
            trendOlderThan4h++;
        if (seen == (1u << 12u))
            trendAgingOut++;
    }
    lastPerHopCounts = counts;

    // 1b. Pick the politeness factor from measured channel utilization.  How far the walk may
    //     stretch and whether it is applied at all now read the same signal, so a node cannot be
    //     told the mesh is filling up by node counts while the channel says it is idle.
    if (smoothedUtilPct() >= CONGESTION_STRICT_PCT)
        lastPoliteNumer = POLITENESS_STRICT;
    else if (smoothedUtilPct() >= CONGESTION_ENGAGE_PCT)
        lastPoliteNumer = POLITENESS_DEFAULT;
    else
        lastPoliteNumer = POLITENESS_GENEROUS;

    // 1c. Scale and cache trend stats (denominatorHistory already advanced above).
    {
        MeshTrendStats t{};
        for (uint8_t h = 0; h < 13; h++) {
            const uint32_t s = static_cast<uint32_t>(hourlyRaw[h]) * denominatorHistory[h];
            t.scaledPerHour[h] = static_cast<uint16_t>(std::min<uint32_t>(s, UINT16_MAX));
        }
        auto scale = [&](uint16_t raw) -> uint16_t {
            return static_cast<uint16_t>(std::min<uint32_t>(static_cast<uint32_t>(raw) * filteringDenominator, UINT16_MAX));
        };
        t.newThisHour = scale(trendNewThisHour);
        t.returningThisHour = scale(trendReturning);
        t.lapsedSinceLastHour = scale(trendLapsed);
        t.olderThan4h = scale(trendOlderThan4h);
        t.agingOut = scale(trendAgingOut);
        lastTrendStats = t;
    }

    // 2. Walk scaled hop buckets to produce a hop-limit recommendation.
    //     effectiveMin: walk threshold - first hop whose cumulative count reaches this.
    //     effectiveMax: ceiling on the one-hop extension check with GENEROUS politeness.
    const uint16_t effectiveMin = TARGET_AFFECTED_NODES;
    const uint16_t effectiveMax = MAX_TARGET_NODES;
    uint8_t suggested = MAX_HOP;
    if (counts.total > 0) {
        uint32_t cumulative = 0;
        for (uint8_t hop = 0; hop <= MAX_HOP; hop++) {
            cumulative += static_cast<uint32_t>(counts.perHop[hop]) * filteringDenominator;
            if (cumulative >= effectiveMin) {
                suggested = hop;
                break;
            }
        }
        if (suggested < MAX_HOP) {
            const uint32_t atNext = static_cast<uint32_t>(counts.perHop[suggested + 1]) * filteringDenominator;
            // politeLimit = effectiveMin + gap * politeNumer / POLITENESS_DENOM
            // Multiply both sides by POLITENESS_DENOM to stay in integers.
            const uint32_t gap = static_cast<uint32_t>(effectiveMax) - static_cast<uint32_t>(effectiveMin);
            if ((cumulative + atNext) * POLITENESS_DENOM <=
                static_cast<uint32_t>(effectiveMin) * POLITENESS_DENOM + gap * lastPoliteNumer) {
                suggested++;
            }
        }
    }
    lastSuggestedHop = suggested;

    // 3. Scale-down check: if fewer than FILL_LOW_PCT% of capacity pass the filteringDenominator
    //    gate and are active, halve samplingDenominator to admit more nodes.
    //    Note: during a filteringDenominator hold period, lowering samplingDenominator does not
    //    immediately improve counts.total (new admissions don't pass the elevated
    //    filteringDenominator).  On a genuinely quieting mesh this check can therefore fire on
    //    consecutive hours, cascading samplingDenominator toward DENOM_MIN.  This is intentional:
    //    rapid re-admission allows quick recovery if the mesh returns.  The hop recommendation
    //    stays conservative (MAX_HOP) throughout because filteringDenominator remains elevated;
    //    step 4 below re-synchronises the denominators once the hold expires.
    if (counts.total * 100u < static_cast<uint32_t>(CAPACITY) * FILL_LOW_PCT) {
        if (samplingDenominator > DENOM_MIN) {
            samplingDenominator = static_cast<uint8_t>(samplingDenominator / 2u);
        }
    }

    // 4. Tick down the hold counter; once it reaches zero, halve filteringDenominator toward
    //    samplingDenominator once per rollHour() (= once per hour) rather than a single jump:
    //    avoids a sudden large change in the hop-walk count when samplingDenominator cascaded
    //    down significantly during the hold period.  No new hold is placed on each step - the
    //    13-roll hold already guaranteed that re-admitted nodes have full seenHoursAgo history;
    //    further pacing is provided naturally by the 1-step-per-hour rate.  denominatorHistory
    //    is updated automatically by the shift at the top of rollHour(), so no backfill here.
    if (filteringDenominator > samplingDenominator) {
        if (filteringDenomHoldRollsRemaining > 0)
            filteringDenomHoldRollsRemaining--;
        if (filteringDenomHoldRollsRemaining == 0) {
            const uint8_t stepped = static_cast<uint8_t>(filteringDenominator / 2u);
            filteringDenominator = (stepped > samplingDenominator) ? stepped : samplingDenominator;
        }
    }

    // 5. Shift all seen bitmaps left by one slot (opens a fresh slot for the new hour).
    for (uint8_t i = 0; i < count; i++) {
        rollSeenBits(entries[i]);
    }

    if (histogramRollCount < 255)
        histogramRollCount++;
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

void HopScalingModule::trimIfNeeded()
{
    // Step 1: evict stale entries (not seen in any of the past 13 hours).
    uint8_t newCount = 0;
    for (uint8_t i = 0; i < count; i++) {
        if (seenInLast13h(entries[i])) {
            if (i != newCount) {
                entries[newCount] = entries[i];
            }
            newCount++;
        }
    }
    this->count = newCount;

    // Step 2: if still too full, double the sampling denominator and remove non-matching entries.
    if (getFillPercentage() >= FILL_HIGH_PCT && samplingDenominator < DENOM_MAX) {
        samplingDenominator = static_cast<uint8_t>(
            std::min<uint16_t>(static_cast<uint16_t>(samplingDenominator) * 2u, static_cast<uint16_t>(DENOM_MAX)));
        filteringDenominator = std::max(filteringDenominator, samplingDenominator);
        filteringDenomHoldRollsRemaining = FILTER_DENOM_HOLD_ROLLS;
        // Raise any denominatorHistory slot that is below the new filteringDenominator.
        // Slots already above it (recorded during a prior scale-up that hasn't fully stepped
        // down yet) are left untouched: eviction at samplingDenominator retains exactly those
        // entries, so the old higher gate remains accurate for those historical hours.
        // Slots below the new value must be raised because the eviction removed entries that
        // had been admitted at the looser old gate - the remaining entries represent a 1/N
        // subsample where N is the new filteringDenominator, not the old smaller value.
        for (uint8_t h = 0; h < 13; h++)
            denominatorHistory[h] = std::max(denominatorHistory[h], filteringDenominator);

        newCount = 0;
        for (uint8_t i = 0; i < count; i++) {
            if (passesFilter(entries[i].nodeHash, samplingDenominator)) {
                if (i != newCount) {
                    entries[newCount] = entries[i];
                }
                newCount++;
            }
        }
        this->count = newCount;
    }
}

float HopScalingModule::channelUtil()
{
#ifdef PIO_UNIT_TESTING
    return s_testChannelUtil;
#else
    return airTime ? airTime->smoothedChannelUtilizationPercent() : 0.0f;
#endif
}

void HopScalingModule::updateCongestion()
{
    // AirTime folds its own EMA once per 10 s bucket, so this reads a figure that already covers
    // the whole interval between ticks rather than only the 60 s before each one.
    utilizationAvg = channelUtil();

    // Separate engage/release thresholds, each confirmed over several ticks, so a mesh sitting
    // near a threshold does not flap the hop limit between rolls.
    const uint8_t util = smoothedUtilPct();
    const bool wantsFlip = congested ? (util <= CONGESTION_RELEASE_PCT) : (util >= CONGESTION_ENGAGE_PCT);
    congestionConfirmRuns = wantsFlip ? static_cast<uint8_t>(congestionConfirmRuns + 1u) : 0u;
    if (congestionConfirmRuns >= CONGESTION_CONFIRM_RUNS) {
        congested = !congested;
        congestionConfirmRuns = 0;
        LOG_INFO("[HOPSCALE] Congestion %s, util %u%%", congested ? "on" : "off", static_cast<unsigned>(utilizationAvg));
    }
}

int32_t HopScalingModule::runOnce()
{
    const bool isFirstRun = !hasCompletedInitialRun;
    bool didHourlyUpdate = false;

    adoptSnapshotIfReady();

    // Sampled every tick, not only on a roll, so the gate reacts within minutes of a change.
    updateCongestion();

    if (isFirstRun) {
        hasCompletedInitialRun = true;
        runsSinceLastHourlyUpdate = 0;
        didHourlyUpdate = true;
    } else {
        runsSinceLastHourlyUpdate++;
        if (runsSinceLastHourlyUpdate >= RUNS_PER_HOUR) {
            runsSinceLastHourlyUpdate = 0;
            didHourlyUpdate = true;
        }
    }

    if (didHourlyUpdate && !isFirstRun) {
        rollHour();
    }

    if (didHourlyUpdate) {
        if (!congested) {
            // Density alone is not a reason to throttle.  Hand a hop back per roll rather than
            // jumping to HOP_MAX, so a mesh that just quietened does not un-throttle all at once.
            if (lastRequiredHop < HOP_MAX)
                lastRequiredHop++;
        } else {
            uint8_t suggested = (histogramRollCount > 0 && count > 0) ? lastSuggestedHop : HOP_MAX;
            // Role-based hop floor: TRACKER/TAK_TRACKER always reach at least 2 hops, SENSOR reaches
            // at least 1, and the infrastructure roles reach INFRASTRUCTURE_HOP_FLOOR, so these
            // reporting roles remain reachable even on a dense mesh recommending fewer hops.
            // The infrastructure set matches the one Router.cpp uses for zero-cost hops.
            uint8_t roleFloor = 0;
            switch (config.device.role) {
            case meshtastic_Config_DeviceConfig_Role_ROUTER:
            case meshtastic_Config_DeviceConfig_Role_ROUTER_LATE:
            case meshtastic_Config_DeviceConfig_Role_CLIENT_BASE:
                roleFloor = INFRASTRUCTURE_HOP_FLOOR;
                break;
            case meshtastic_Config_DeviceConfig_Role_TRACKER:
            case meshtastic_Config_DeviceConfig_Role_TAK_TRACKER:
                roleFloor = 2;
                break;
            case meshtastic_Config_DeviceConfig_Role_SENSOR:
                roleFloor = 1;
                break;
            default:
                break;
            }
            lastRequiredHop = std::max(suggested, roleFloor);
        }
    }

    LOG_INFO("[HOPSCALE] hop=%u util=%u%% nodes=%u samp=1/%u filt=1/%u", lastRequiredHop, static_cast<unsigned>(utilizationAvg),
             count, samplingDenominator, filteringDenominator);

    return RUN_INTERVAL_MS;
}

#endif
