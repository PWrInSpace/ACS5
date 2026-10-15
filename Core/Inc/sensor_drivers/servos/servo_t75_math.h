/*
 * SRT Servo T75 HV platform-independent math: angle->pulse mapping (trim,
 * direction, clamping) and slew-rate-limited stepping. Host-testable.
 * Datasheet: neutral 1500 us, 90 deg span 1000-2000 us => 11.111 us/deg.
 */
#pragma once

#include <stdint.h>

typedef struct {
    uint16_t neutral_us;
    uint16_t min_us;
    uint16_t max_us;
    int8_t   direction_sign; /* +1 or -1 */
    float    us_per_deg;
} servo_t75_limits_t;

/* Clamp deg to +/-max_angle_deg, apply direction, then clamp the result to
 * [min_us, max_us] last - output is always inside the hard limits. */
uint16_t servo_t75_angle_to_pulse_us(float deg, const servo_t75_limits_t *lim, float max_angle_deg);

/* Move current one tick toward target, bounded by us_per_ms*dt_ms; never overshoots. */
uint16_t servo_t75_slew_step(uint16_t current, uint16_t target, float us_per_ms, uint32_t dt_ms);
