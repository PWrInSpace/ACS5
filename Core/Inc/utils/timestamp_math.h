/*
 * Pure 32->64-bit counter extension for the DWT cycle counter, split from
 * timestamp.h so host tests compile it without CMSIS headers.
 * Correct while consecutive calls are less than one 2^32-count wrap apart
 * (~14.3 s at 300 MHz); the 500 Hz loop calls every 2 ms.
 */
#pragma once

#include <stdint.h>

typedef struct {
    uint32_t last_count;
    uint64_t total;
} counter64_t;

static inline uint64_t counter64_extend(counter64_t *c, uint32_t now)
{
    c->total     += (uint32_t)(now - c->last_count);
    c->last_count = now;
    return c->total;
}
