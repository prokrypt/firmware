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

Scope::Scope(const char *reason) : previousReason(currentReason)
{
    currentReason = reason;
    depth++;
}

Scope::~Scope()
{
    depth--;
    currentReason = previousReason;
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
