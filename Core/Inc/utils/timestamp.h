/*
 * High-resolution monotonic timestamp via the Cortex-M7 DWT cycle counter.
 * Call timestamp_init() once at boot before any read. Raw CYCCNT wraps every
 * 2^32 cycles (~14.3 s @ 300 MHz); timestamp_cycles64() extends it in
 * software, so timestamp_us() is subtraction-safe for spans up to ~71 min.
 * The extension state is unguarded: call timestamp_us()/timestamp_cycles64()
 * from the aggregator task only.
 */
#pragma once

#include <stdint.h>
#include "main.h"
#include "utils/timestamp_math.h"

extern counter64_t g_cycle_counter;

/* DWT CYCCNT runs at the CPU clock. Kept as a compile-time constant so the
 * division in timestamp_us() folds to a multiply instead of __aeabi_uldivmod;
 * timestamp_init() traps a clock tree that no longer matches. */
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
