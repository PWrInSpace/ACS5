#include "servos/servo_t75_math.h"
#include "utils/mathx.h"

#include <math.h>
#include <stdlib.h>

uint16_t servo_t75_angle_to_pulse_us(float deg, const servo_t75_limits_t *lim, float max_angle_deg)
{
    max_angle_deg = fabsf(max_angle_deg);
    deg = clampf(deg, -max_angle_deg, max_angle_deg);

    const float sign      = (lim->direction_sign >= 0) ? 1.0f : -1.0f;
    const float offset_us = sign * deg * lim->us_per_deg;

    /* Round before clamping so the hard microsecond limits always win. */
    const float pulse = roundf((float)lim->neutral_us + offset_us);

    return (uint16_t)clampf(pulse, (float)lim->min_us, (float)lim->max_us);
}

uint16_t servo_t75_slew_step(uint16_t current, uint16_t target, float us_per_ms, uint32_t dt_ms)
{
    if (current == target || dt_ms == 0 || us_per_ms <= 0.0f)
        return current;

    const float max_delta = us_per_ms * (float)dt_ms;
    const int   diff      = (int)target - (int)current;

    if ((float)abs(diff) <= max_delta)
        return target;

    const float step        = (diff > 0) ? max_delta : -max_delta;
    const float new_pulse    = (float)current + step;
    const int   new_pulse_i  = (int)roundf(new_pulse);

    if (new_pulse_i < 0)
        return 0;
    if (new_pulse_i > 0xFFFF)
        return 0xFFFF;
    return (uint16_t)new_pulse_i;
}
