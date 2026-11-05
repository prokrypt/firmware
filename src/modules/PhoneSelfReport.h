#pragma once
#include <stdint.h>

// Cadence for the phone-only copies of our own position/telemetry: fast when the value changed, else a slow refresh.
namespace PhoneSelfReport
{
constexpr uint32_t CHANGED_INTERVAL_MS = 60 * 1000;
constexpr uint32_t UNCHANGED_INTERVAL_MS = 15 * 60 * 1000;

constexpr uint32_t intervalMs(bool changed)
{
    return changed ? CHANGED_INTERVAL_MS : UNCHANGED_INTERVAL_MS;
}

// Movement below this since the last phone copy is treated as GPS jitter.
constexpr float POSITION_CHANGE_METERS = 25.0f;
} // namespace PhoneSelfReport
