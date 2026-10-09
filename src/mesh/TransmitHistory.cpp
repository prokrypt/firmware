#include "TransmitHistory.h"
#include "UptimeClock.h"

#ifdef ARCH_ESP32
#include "sleep.h"
#endif

TransmitHistory *transmitHistory = nullptr;

TransmitHistory *TransmitHistory::getInstance()
{
    if (!transmitHistory) {
        transmitHistory = new TransmitHistory();
    }
    return transmitHistory;
}

void TransmitHistory::setLastSentToMesh(uint16_t key)
{
    lastMillis[key] = Time::skipZero(Time::getMillis());
}

uint32_t TransmitHistory::getLastSentToMeshMillis(uint16_t key) const
{
    auto it = lastMillis.find(key);
    return (it != lastMillis.end()) ? it->second : 0;
}

bool TransmitHistory::wokeFromDeepSleep() const
{
#ifdef PIO_UNIT_TESTING
    if (wokeFromDeepSleepOverride >= 0)
        return wokeFromDeepSleepOverride == 1;
#endif
#ifdef ARCH_ESP32
    return wakeCause != ESP_SLEEP_WAKEUP_UNDEFINED;
#else
    return false;
#endif
}

uint32_t TransmitHistory::bootHoldoffRemainingMs() const
{
    if (wokeFromDeepSleep())
        return 0;
    // 64-bit uptime: the holdoff must not reopen when the 32-bit millis() wraps after ~49.7 days.
    const uint64_t uptimeMs = Time::getMillisMonotonic();
    return uptimeMs >= BOOT_HOLDOFF_MS ? 0 : (uint32_t)(BOOT_HOLDOFF_MS - uptimeMs);
}

bool TransmitHistory::inBootHoldoff() const
{
    return bootHoldoffRemainingMs() > 0;
}

void TransmitHistory::clear()
{
    lastMillis.clear();
}
