// The loop wakes on osDelayUntil against a 1 kHz FreeRTOS tick, so its wall
// period is a whole number of milliseconds. Everything the control code calls
// "seconds" -- CONTROL_DT_S fed to integrators, SEC_TO_TICKS timeouts,
// HZ_TO_TICKS cadences -- is only true if it agrees with that period. These
// assertions pin that agreement; a rate the scheduler cannot produce breaks
// them here instead of silently rescaling every timer in flight.
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "control/control_rate.h"

/* How aggregator.c derives the osDelayUntil step. */
#define WALL_TICK_MS ((uint32_t)(1000.0f / CONTROL_LOOP_HZ))

static void test_dt_matches_wall_period(void)
{
    assert(WALL_TICK_MS >= 1);                    /* faster than the RTOS tick is unschedulable */
    assert(fabsf(CONTROL_DT_S * 1000.0f - (float)WALL_TICK_MS) < 1e-6f);
}

static void test_rate_is_schedulable(void)
{
    /* Truncation in the line above is what turns a nominal rate into a
     * different real one, so require the rate to divide 1 kHz exactly. */
    assert(fabsf(1000.0f / CONTROL_LOOP_HZ - (float)WALL_TICK_MS) < 1e-6f);
}

static void test_sec_to_ticks_lands_on_wall_time(void)
{
    const float secs[] = {0.016f, 0.040f, 0.5f, 6.0f, 240.0f};

    for (unsigned i = 0; i < sizeof secs / sizeof *secs; i++) {
        float wall_ms = (float)SEC_TO_TICKS(secs[i]) * (float)WALL_TICK_MS;
        assert(fabsf(wall_ms - secs[i] * 1000.0f) <= (float)WALL_TICK_MS);
    }
}

static void test_hz_to_ticks_lands_on_wall_time(void)
{
    const float hz[] = {12.5f, 50.0f};

    for (unsigned i = 0; i < sizeof hz / sizeof *hz; i++) {
        float wall_ms = (float)HZ_TO_TICKS(hz[i]) * (float)WALL_TICK_MS;
        assert(fabsf(wall_ms - 1000.0f / hz[i]) <= (float)WALL_TICK_MS);
    }
}

int main(void)
{
    test_dt_matches_wall_period();
    test_rate_is_schedulable();
    test_sec_to_ticks_lands_on_wall_time();
    test_hz_to_ticks_lands_on_wall_time();
    printf("control_rate: OK\n");
    return 0;
}
