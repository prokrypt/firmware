#pragma once
#include <cstdint>

// Flash writes are only legitimate inside a Scope opened by a user action or a sanctioned one-time write.
// Unscoped writes are logged and counted; FLASH_GUARD_ENFORCE refuses them.
namespace FlashGuard
{

class Scope
{
  public:
    explicit Scope(const char *reason);
    ~Scope();
    Scope(const Scope &) = delete;
    Scope &operator=(const Scope &) = delete;

  private:
    const char *previousReason;
    uint32_t watchedAtEntry;
};

bool inScope();

/// Call immediately before any flash mutation (write, erase, remove, format, NVS put).
/// Returns false if the write must not proceed.
bool noteWrite(const char *what);

/// Latch into *latch the value *watched holds whenever a Scope changes it. Lets code that acts later (a
/// scheduled reboot) tell whether a user action scheduled it. One watch; a second call replaces the first.
void watchForUserChange(const volatile uint32_t *watched, uint32_t *latch);

uint32_t writesSinceBoot();
uint32_t unscopedWritesSinceBoot();

} // namespace FlashGuard
