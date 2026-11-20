#pragma once

#include <stdint.h>

/* The single statement of the control-loop rate; derive every rate-dependent count via SEC_TO_TICKS/HZ_TO_TICKS.
 * Why the period, not the rate, is primary: docs/design.md#control-loop-rate */
#define CONTROL_TICK_MS 2u
#define CONTROL_LOOP_HZ (1000.0f / (float)CONTROL_TICK_MS)
#define CONTROL_DT_S    ((float)CONTROL_TICK_MS / 1000.0f)

/* Rounded to nearest rather than truncated toward zero. */
#define SEC_TO_TICKS(s)  ((uint32_t)((s) * CONTROL_LOOP_HZ + 0.5f))
#define HZ_TO_TICKS(hz)  ((uint32_t)(CONTROL_LOOP_HZ / (hz) + 0.5f))
