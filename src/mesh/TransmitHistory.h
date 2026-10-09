#pragma once

#include "configuration.h"
#include <Arduino.h>
#include <map>

/**
 * TransmitHistory tracks, in RAM only, when we last broadcast each kind of packet (keyed by portnum or a
 * module-private key), so modules can throttle against it. Nothing is written to flash: after a reboot every
 * key reads as never sent.
 *
 * In its place, a boot holdoff stops our own periodic broadcasts for BOOT_HOLDOFF_MS after a cold boot, so a
 * reboot loop cannot flood the mesh. Sends a user triggers (phone, admin, menus, buttons) do not check it, and
 * neither does a wake from deep sleep (ESP32): that is a scheduled duty cycle, a button press or a packet, not a
 * reboot loop.
 */

#include "mesh/generated/meshtastic/portnums.pb.h"

class TransmitHistory
{
  public:
    /// How long our periodic broadcasts wait after a cold boot.
    static constexpr uint32_t BOOT_HOLDOFF_MS = 5 * 60 * 1000;

    static TransmitHistory *getInstance();

    /// Record that a broadcast was sent for the given key right now.
    void setLastSentToMesh(uint16_t key);

    /// millis()-relative time of the last send for this key this boot, or 0 if never sent.
    uint32_t getLastSentToMeshMillis(uint16_t key) const;

    /// True while periodic broadcasts must wait out the boot holdoff. User-initiated sends ignore it.
    bool inBootHoldoff() const;

    /// Milliseconds left in the boot holdoff; 0 once it is over or when it does not apply.
    uint32_t bootHoldoffRemainingMs() const;

    /// Forget every stamp (factory reset).
    void clear();

#ifdef PIO_UNIT_TESTING
    /// Pretend this boot was (or was not) a wake from deep sleep.
    void setWokeFromDeepSleepForTest(bool woke) { wokeFromDeepSleepOverride = woke ? 1 : 0; }
#endif

  private:
    TransmitHistory() = default;
    bool wokeFromDeepSleep() const;

    std::map<uint16_t, uint32_t> lastMillis; // key -> millis() of the last send this boot
#ifdef PIO_UNIT_TESTING
    int8_t wokeFromDeepSleepOverride = -1; // -1 = ask the platform
#endif
};

extern TransmitHistory *transmitHistory;
