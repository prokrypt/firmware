#include "WarmNodeStore.h"

#if WARM_NODE_COUNT > 0

#include "FSCommon.h"
#include "FlashGuard.h"
#include "SPILock.h"
#include "configuration.h"
#include "memory/MemAudit.h"

#if defined(NRF52840_XXAA)
#include "flash/flash_nrf5x.h"
#elif defined(FSCom)
static const char *warmFileName = "/prefs/warm.dat";
#endif

static inline bool keyIsSet(const uint8_t key[32])
{
    for (int i = 0; i < 32; i++)
        if (key[i])
            return true;
    return false;
}

WarmNodeStore::WarmNodeStore()
{
#if defined(ARCH_ESP32) && defined(BOARD_HAS_PSRAM)
    entries = static_cast<WarmNodeEntry *>(ps_calloc(WARM_NODE_COUNT, sizeof(WarmNodeEntry)));
    if (!entries) {
        LOG_WARN("WarmStore: PSRAM alloc failed, using heap");
        entries = static_cast<WarmNodeEntry *>(calloc(WARM_NODE_COUNT, sizeof(WarmNodeEntry)));
    }
#else
    entries = static_cast<WarmNodeEntry *>(calloc(WARM_NODE_COUNT, sizeof(WarmNodeEntry)));
#endif
    memaudit::set("warm", entries ? WARM_NODE_COUNT * sizeof(WarmNodeEntry) : 0);
}

WarmNodeStore::~WarmNodeStore()
{
    free(entries); // always malloc-family (calloc / ps_calloc)
    entries = nullptr;
    memaudit::set("warm", 0);
}

WarmNodeEntry *WarmNodeStore::find(NodeNum num) const
{
    if (!entries || !num)
        return nullptr;
    for (size_t i = 0; i < WARM_NODE_COUNT; i++)
        if (entries[i].num == num)
            return &entries[i];
    return nullptr;
}

// Slot placement with the keyed-first admission policy. Shared by absorb()
// and the ring replay, so the policy is applied identically in both paths.
WarmNodeEntry *WarmNodeStore::place(NodeNum num, uint32_t lastHeard, const uint8_t *key32)
{
    if (!entries || !num)
        return nullptr;

    const bool candidateKeyed = key32 && keyIsSet(key32);

    WarmNodeEntry *slot = find(num);
    const bool sameNode = slot != nullptr;
    if (!slot) {
        // Pick a victim: any empty slot, else the oldest keyless entry, else
        // (only for keyed candidates) the oldest keyed entry.
        WarmNodeEntry *oldestKeyless = nullptr, *oldestKeyed = nullptr;
        for (size_t i = 0; i < WARM_NODE_COUNT; i++) {
            WarmNodeEntry &e = entries[i];
            if (!e.num) {
                slot = &e;
                break;
            }
            // Compare on the time bits only - the low metadata bits (role/protected) must
            // not perturb LRU victim selection.
            if (keyIsSet(e.public_key)) {
                if (!oldestKeyed || warmTimeOf(e) < warmTimeOf(*oldestKeyed))
                    oldestKeyed = &e;
            } else {
                if (!oldestKeyless || warmTimeOf(e) < warmTimeOf(*oldestKeyless))
                    oldestKeyless = &e;
            }
        }
        if (!slot)
            slot = oldestKeyless ? oldestKeyless : (candidateKeyed ? oldestKeyed : nullptr);
        if (!slot)
            return nullptr; // store full of keyed entries and the candidate has no key
    }

    slot->num = num;
    slot->last_heard = lastHeard;
    if (candidateKeyed)
        memcpy(slot->public_key, key32, 32);
    else if (!sameNode)
        // Repurposing a victim slot for a different node: clear its stale key.
        // A keyless refresh of a node already here keeps the key we learned.
        memset(slot->public_key, 0, 32);
    return slot;
}

bool WarmNodeStore::absorb(NodeNum num, uint32_t lastHeard, const uint8_t *key32, uint8_t role, uint8_t protectedCat,
                           bool xeddsaSigned)
{
    // Pack role + protected category + xeddsa-signed into the low bits of last_heard. place() and
    // ring replay store the raw word verbatim, so the metadata round-trips through flash.
    const uint32_t packed = warmPackLastHeard(lastHeard, role, protectedCat, xeddsaSigned);
    const WarmNodeEntry *slot = place(num, packed, key32);
    if (!slot)
        return false;
    LOG_MIGRATION("WarmStore absorb 0x%08x key=%d last_heard=%u role=%u prot=%u xeddsa=%u (now %u/%u)", (unsigned)num,
                  keyIsSet(slot->public_key) ? 1 : 0, (unsigned)warmTimeOf(*slot), (unsigned)role, (unsigned)protectedCat,
                  xeddsaSigned ? 1u : 0u, (unsigned)count(), (unsigned)capacity());
    return true;
}

bool WarmNodeStore::lookupMeta(NodeNum num, uint8_t &role, uint8_t &protectedCat) const
{
    const WarmNodeEntry *e = find(num);
    if (!e)
        return false;
    role = warmRoleOf(*e);
    protectedCat = warmProtOf(*e);
    return true;
}

bool WarmNodeStore::hasXeddsaSigned(NodeNum num) const
{
    const WarmNodeEntry *e = find(num);
    return e && warmXeddsaSignedOf(*e);
}

bool WarmNodeStore::take(NodeNum num, WarmNodeEntry &out)
{
    WarmNodeEntry *e = find(num);
    if (!e)
        return false;
    out = *e;
    memset(e, 0, sizeof(*e));
    LOG_MIGRATION("WarmStore take(rehydrate) 0x%08x key=%d (now %u/%u)", (unsigned)num, keyIsSet(out.public_key) ? 1 : 0,
                  (unsigned)count(), (unsigned)capacity());
    return true;
}

#if MESHTASTIC_NODEDB_MIGRATION_VERBOSE
void WarmNodeStore::dumpToLog(const char *reason) const
{
    if (!entries) {
        LOG_MIGRATION("WarmStore dump (%s): backend not allocated", reason);
        return;
    }
    LOG_MIGRATION("WarmStore dump (%s): %u live / %u cap ==>", reason, (unsigned)count(), (unsigned)capacity());
    unsigned shown = 0;
    for (size_t i = 0; i < WARM_NODE_COUNT; i++) {
        const WarmNodeEntry &e = entries[i];
        if (e.num == 0)
            continue;
        LOG_MIGRATION("  warm[%3u] 0x%08x last_heard=%u key=%d", (unsigned)i, (unsigned)e.num, (unsigned)e.last_heard,
                      keyIsSet(e.public_key) ? 1 : 0);
        shown++;
    }
    LOG_MIGRATION("WarmStore dump (%s): <== end (%u entries)", reason, shown);
}
#endif // MESHTASTIC_NODEDB_MIGRATION_VERBOSE

bool WarmNodeStore::copyKey(NodeNum num, uint8_t out[32]) const
{
    const WarmNodeEntry *e = find(num);
    if (!e || !keyIsSet(e->public_key))
        return false;
    memcpy(out, e->public_key, 32);
    return true;
}

bool WarmNodeStore::contains(NodeNum num) const
{
    return find(num) != nullptr;
}

void WarmNodeStore::remove(NodeNum num)
{
    WarmNodeEntry *e = find(num);
    if (e) {
        memset(e, 0, sizeof(*e));
    }
}

void WarmNodeStore::clear()
{
    if (!entries)
        return;
    memset(entries, 0, WARM_NODE_COUNT * sizeof(WarmNodeEntry));
}

size_t WarmNodeStore::count() const
{
    size_t n = 0;
    if (entries)
        for (size_t i = 0; i < WARM_NODE_COUNT; i++)
            if (entries[i].num)
                n++;
    return n;
}

#if defined(NRF52840_XXAA)
namespace
{
// The flash_nrf5x page cache is shared with LittleFS, whose writers hold only this mutex. spiLock first.
struct WarmFsLock {
    WarmFsLock() { FSCom._lockFS(); }
    ~WarmFsLock() { FSCom._unlockFS(); }
};
} // namespace
#endif

void WarmNodeStore::eraseFlash()
{
#if defined(NRF52840_XXAA)
    concurrency::LockGuard g(spiLock);
    WarmFsLock fs;
    flash_nrf5x_flush();
    for (uint8_t p = 0; p < WARM_FLASH_PAGES; p++) {
        uint32_t header[2];
        flash_nrf5x_read(header, WARM_FLASH_PAGE_ADDR(p), sizeof(header));
        if (header[0] == 0xFFFFFFFFu && header[1] == 0xFFFFFFFFu)
            continue; // already erased; skip the erase cycle
        if (!FlashGuard::noteWrite("warm ring erase"))
            return;
        flash_nrf5x_erase(WARM_FLASH_PAGE_ADDR(p));
    }
#elif defined(FSCom)
    concurrency::LockGuard g(spiLock);
    if (FSCom.exists(warmFileName) && FlashGuard::noteWrite(warmFileName))
        FSCom.remove(warmFileName);
#endif
}

#endif // WARM_NODE_COUNT > 0
