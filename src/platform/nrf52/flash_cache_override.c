/*
 * Replaces libraries/InternalFileSytem/src/flash/flash_cache.c of the Adafruit nRF52 core (MIT, Copyright (c)
 * 2019 Ha Thach for Adafruit Industries). The core's flash_cache_flush() erases a 4 KB page and programs it,
 * then forgets the result: an erase or program that fails after fal_*'s retries leaves the page blank or
 * half written, and the LittleFS callbacks still report success. A torn metadata pair is how /prefs was lost
 * on a T1000-E (2026-10-28). This version verifies the page after programming, retries the whole
 * erase+program a few times, and counts the flushes that still failed so SafeFile can fail the save.
 *
 * Defining all three symbols here keeps the linker from pulling the core's flash_cache.o out of the
 * framework archive. flash_cache_write() and flash_cache_read() are the core's code, unchanged.
 */

#include "common_func.h"
#include "flash/flash_cache.h"
#include "variant.h"
#include "wiring_digital.h"
#include <string.h>

#define FLASH_CACHE_FLUSH_ATTEMPTS 3

/// Pages that could not be written back even after the retries. Read by SafeFile::close().
volatile uint32_t flash_cache_flush_failures = 0;

static inline uint32_t page_addr_of(uint32_t addr)
{
    return addr & ~(FLASH_CACHE_SIZE - 1);
}

static inline uint32_t page_offset_of(uint32_t addr)
{
    return addr & (FLASH_CACHE_SIZE - 1);
}

int flash_cache_write(flash_cache_t *fc, uint32_t dst, void const *src, uint32_t len)
{
    uint8_t const *src8 = (uint8_t const *)src;
    uint32_t remain = len;

    // Program up to page boundary each loop
    while (remain) {
        uint32_t const page_addr = page_addr_of(dst);
        uint32_t const offset = page_offset_of(dst);

        uint32_t wr_bytes = FLASH_CACHE_SIZE - offset;
        wr_bytes = min32(remain, wr_bytes);

        // Page changes, flush old and update new cache
        if (page_addr != fc->cache_addr) {
            flash_cache_flush(fc);
            fc->cache_addr = page_addr;

            // read a whole page from flash
            fc->read(fc->cache_buf, page_addr, FLASH_CACHE_SIZE);
        }

        memcpy(fc->cache_buf + offset, src8, wr_bytes);

        // adjust for next run
        src8 += wr_bytes;
        remain -= wr_bytes;
        dst += wr_bytes;
    }

    return len - remain;
}

void flash_cache_flush(flash_cache_t *fc)
{
    if (fc->cache_addr == FLASH_CACHE_INVALID_ADDR)
        return;

    // skip erase & program if memory already matches
    bool ok = fc->verify(fc->cache_addr, fc->cache_buf, FLASH_CACHE_SIZE);
    for (int attempt = 0; !ok && attempt < FLASH_CACHE_FLUSH_ATTEMPTS; attempt++) {
        ledOn(LED_BUILTIN);
        ok = fc->erase(fc->cache_addr) && fc->program(fc->cache_addr, fc->cache_buf, FLASH_CACHE_SIZE) == FLASH_CACHE_SIZE &&
             fc->verify(fc->cache_addr, fc->cache_buf, FLASH_CACHE_SIZE);
        ledOff(LED_BUILTIN);
    }
    if (!ok)
        flash_cache_flush_failures++;

    fc->cache_addr = FLASH_CACHE_INVALID_ADDR;
}

int flash_cache_read(flash_cache_t *fc, void *dst, uint32_t addr, uint32_t count)
{
    // overwrite with cache value if available
    if ((fc->cache_addr != FLASH_CACHE_INVALID_ADDR) &&               // cache is not valid
        !(addr < fc->cache_addr && addr + count <= fc->cache_addr) && // starts before, ends before cache area
        !(addr >= fc->cache_addr + FLASH_CACHE_SIZE))                 // starts after end of cache area
    {
        // The read overlaps the cache area by at least one byte. At most one of dst_off (bytes read from
        // flash before the cache area) and src_off (offset into the cache area) is non-zero.
        uint32_t dst_off = 0;
        uint32_t src_off = 0;
        if (addr < fc->cache_addr) {
            dst_off = fc->cache_addr - addr;
            fc->read(dst, addr, dst_off);
        } else {
            src_off = addr - fc->cache_addr;
        }

        uint32_t cache_bytes = minof(FLASH_CACHE_SIZE - src_off, count - dst_off);
        memcpy(dst + dst_off, fc->cache_buf + src_off, cache_bytes);

        // Read any final bytes from flash
        uint32_t copied = dst_off + cache_bytes;
        if (copied < count) {
            fc->read(dst + copied, addr + copied, count - copied);
        }
    } else {
        // not using the cache, so just forward to read from flash
        fc->read(dst, addr, count);
    }

    return (int)count;
}
