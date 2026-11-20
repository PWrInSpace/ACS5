/* HAL-free 32->64-bit counter extension; correct while calls are under one 2^32-count
 * wrap apart (~14.3 s at 300 MHz). */
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
