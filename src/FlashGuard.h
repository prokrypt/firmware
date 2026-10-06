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
};

bool inScope();

/// Call immediately before any flash mutation (write, erase, remove, format, NVS put).
/// Returns false if the write must not proceed.
bool noteWrite(const char *what);

uint32_t writesSinceBoot();
uint32_t unscopedWritesSinceBoot();

} // namespace FlashGuard
