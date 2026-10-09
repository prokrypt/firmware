#include "FlashGuard.h"
#include "DebugConfiguration.h"
#include <atomic>

namespace FlashGuard
{

// Depth is global, not per-task: a write on another task while a scope is open counts as scoped.
static std::atomic<int> depth{0};
static const char *currentReason = nullptr;
static std::atomic<uint32_t> totalWrites{0};
static std::atomic<uint32_t> unscopedWrites{0};
static const volatile uint32_t *watchedValue = nullptr;
static uint32_t *watchedLatch = nullptr;

Scope::Scope(const char *reason) : previousReason(currentReason), watchedAtEntry(watchedValue ? *watchedValue : 0)
{
    currentReason = reason;
    depth++;
}

Scope::~Scope()
{
    if (watchedValue && watchedLatch && *watchedValue != watchedAtEntry)
        *watchedLatch = *watchedValue;
    depth--;
    currentReason = previousReason;
}

void watchForUserChange(const volatile uint32_t *watched, uint32_t *latch)
{
    watchedValue = watched;
    watchedLatch = latch;
}

bool inScope()
{
    return depth.load() > 0;
}

bool noteWrite(const char *what)
{
    totalWrites++;
    if (inScope()) {
        LOG_DEBUG("FlashGuard: write %s (%s)", what, currentReason ? currentReason : "?");
        return true;
    }
    unscopedWrites++;
#ifdef FLASH_GUARD_ENFORCE
    LOG_ERROR("FlashGuard: refused unscoped write %s (#%u)", what, (unsigned)unscopedWrites.load());
    return false;
#else
    LOG_WARN("FlashGuard: unscoped write %s (#%u)", what, (unsigned)unscopedWrites.load());
    return true;
#endif
}

uint32_t writesSinceBoot()
{
    return totalWrites.load();
}

uint32_t unscopedWritesSinceBoot()
{
    return unscopedWrites.load();
}

} // namespace FlashGuard
