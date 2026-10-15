#pragma once

#include <stdint.h>

/* The single statement of the control-loop rate. The aggregator derives its
 * tick period from it, the estimator and the roll controller default their
 * dt_s to it, and the host tests integrate their plants on it.
 *
 * Everything rate-dependent derives from it through SEC_TO_TICKS/HZ_TO_TICKS:
 * the aggregator's loop-counter intervals and the tick-denominated debounces
 * in flight_sm and roll_ctrl. Adding a new one as a bare literal reintroduces
 * a constant that a rate change would silently rescale.
 *
 * The period is primary, not the rate. The loop wakes on osDelayUntil against a
 * 1 kHz FreeRTOS tick, so only whole-millisecond periods are schedulable, and dt
 * has to be the period actually slept - every integrator, timeout and cadence
 * here is denominated in it. Deriving the rate from CONTROL_TICK_MS keeps the
 * two in step; a rate chosen on its own can be one the scheduler rounds away,
 * leaving dt describing a loop that does not exist. */
#define CONTROL_TICK_MS 2u
#define CONTROL_LOOP_HZ (1000.0f / (float)CONTROL_TICK_MS)
#define CONTROL_DT_S    ((float)CONTROL_TICK_MS / 1000.0f)

/* Rounded to nearest so a rate change lands on the closest achievable tick
 * count instead of truncating toward zero. */
#define SEC_TO_TICKS(s)  ((uint32_t)((s) * CONTROL_LOOP_HZ + 0.5f))
#define HZ_TO_TICKS(hz)  ((uint32_t)(CONTROL_LOOP_HZ / (hz) + 0.5f))
