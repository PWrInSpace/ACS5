#pragma once

/* Pure-C math helpers shared by the control modules and sensor drivers.
 * No HAL/RTOS dependencies, so host tests compile against this directly. */

#define GRAVITY 9.796f     /* local g used by the Mahony-tuned filter, m/s^2 */
#define RAD2DEG 57.2957795f
#define PI_F    3.14159265358979f

static inline float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}
