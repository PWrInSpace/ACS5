/* Monotonic DWT-cycle timestamp, extended to 64 bits; timestamp_init() once at boot first.
 * The extension state is unguarded: read it from the flight task only. */
#pragma once

#include <stdint.h>
#include "main.h"
#include "utils/timestamp_math.h"

extern counter64_t g_cycle_counter;

/* DWT CYCCNT runs at the CPU clock; a compile-time constant lets timestamp_us() divide by
 * multiplying. timestamp_init() traps a clock tree that no longer matches. */
#define CPU_HZ        300000000UL
#define CYCLES_PER_US (CPU_HZ / 1000000UL)

void timestamp_init(void);

static inline uint32_t timestamp_cycles(void)
{
    return DWT->CYCCNT;
}

static inline uint64_t timestamp_cycles64(void)
{
    return counter64_extend(&g_cycle_counter, DWT->CYCCNT);
}

static inline uint32_t timestamp_us(void)
{
    return (uint32_t)(timestamp_cycles64() / CYCLES_PER_US);
}
