// Golden tests for the servo_t75 math: fixed datasheet-derived anchors for the
// angle->pulse mapping and slew stepping.
#include "test_assert.h"
#include <stdint.h>
#include <stdio.h>

#include "sensor_drivers/servos/servo_t75_math.h"

static void test_angle_to_pulse(void) {
    const servo_t75_limits_t lim = {1500, 1300, 1700, +1, 11.111f};

    // Neutral and symmetric mid-range mapping (11.111 us/deg).
    CHECK(servo_t75_angle_to_pulse_us(0.0f, &lim, 15.0f) == 1500);
    CHECK(servo_t75_angle_to_pulse_us(9.0f, &lim, 15.0f) == 1600);
    CHECK(servo_t75_angle_to_pulse_us(-9.0f, &lim, 15.0f) == 1400);

    // Angle clamp to +/-max_angle_deg: 15 deg -> 1500 + 15*11.111 = 1666.7 -> 1667.
    CHECK(servo_t75_angle_to_pulse_us(100.0f, &lim, 15.0f) == 1667);
    CHECK(servo_t75_angle_to_pulse_us(-100.0f, &lim, 15.0f) == 1333);

    // Direction sign flips the deflection.
    const servo_t75_limits_t rev = {1500, 1300, 1700, -1, 11.111f};
    CHECK(servo_t75_angle_to_pulse_us(15.0f, &rev, 15.0f) == 1333);

    // Hard us limit wins over the angle-derived pulse (max_us < angle result).
    const servo_t75_limits_t tight = {1500, 1400, 1600, +1, 11.111f};
    CHECK(servo_t75_angle_to_pulse_us(15.0f, &tight, 15.0f) == 1600);
    CHECK(servo_t75_angle_to_pulse_us(-15.0f, &tight, 15.0f) == 1400);
}

static void test_slew_step(void) {
    // us_per_ms = 2.0 => max 20 us per 10 ms tick.
    CHECK(servo_t75_slew_step(1500, 1600, 2.0f, 10) == 1520);   // step up, rate-limited
    CHECK(servo_t75_slew_step(1500, 1400, 2.0f, 10) == 1480);   // step down, rate-limited
    CHECK(servo_t75_slew_step(1500, 1600, 2.0f, 100) == 1600);  // within reach -> snap to target
    CHECK(servo_t75_slew_step(1500, 1500, 2.0f, 10) == 1500);   // already there
    CHECK(servo_t75_slew_step(1500, 1600, 2.0f, 0) == 1500);    // no time elapsed -> hold
}

int main(void) {
    test_angle_to_pulse();
    test_slew_step();
    return test_report("servo_t75_math");
}
