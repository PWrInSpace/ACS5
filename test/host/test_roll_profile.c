// Trapezoidal roll-angle profile: ramp at max_accel to max_rate, cruise,
// brake to arrive at target with ~zero rate. Analytic duration for
// 90 deg @ 105 deg/s, 210 deg/s^2: 2*0.5 + (1.571 - 2*0.458)/1.833 ~= 1.36 s.
#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "control/roll_ctrl.h"
#include "control/control_rate.h"
#include "utils/mathx.h"

#define DT       CONTROL_DT_S
#define TARGET   (PI_F / 2.0f)
#define MAX_RATE 1.8326f     /* 105 deg/s */
#define MAX_ACC  3.6652f     /* 210 deg/s^2 */

static void test_reaches_target_in_analytic_time(void)
{
    roll_profile_t p;
    roll_profile_init(&p, TARGET, MAX_RATE, MAX_ACC);

    int ticks = 0;
    while (!p.done && ticks < (int)(CONTROL_LOOP_HZ * 10.0f)) { roll_profile_step(&p, DT); ticks++; }

    float t = (float)ticks * DT;
    assert(p.done);
    assert(t > 1.2f && t < 1.6f);                 /* ~1.36 s analytic */
    assert(fabsf(p.theta - TARGET) < 1e-3f);      /* snapped to target */
    assert(fabsf(p.rate) < 1e-3f);
}

static void test_limits_respected_and_monotonic(void)
{
    roll_profile_t p;
    roll_profile_init(&p, TARGET, MAX_RATE, MAX_ACC);

    float prev_theta = 0.0f, prev_rate = 0.0f;
    while (!p.done) {
        roll_profile_step(&p, DT);
        assert(p.rate <= MAX_RATE + 1e-4f && p.rate >= -1e-4f);
        /* The terminating tick snaps a sub-1e-3 rate residual to zero on top of
         * its slew step; every earlier tick is bounded by the slew alone. */
        float rate_budget = MAX_ACC * DT + (p.done ? 1e-3f : 1e-4f);
        assert(fabsf(p.rate - prev_rate) <= rate_budget);
        assert(p.theta >= prev_theta - 1e-5f);           /* monotonic rise */
        assert(p.theta <= TARGET + 0.02f);               /* no overshoot   */
        prev_theta = p.theta; prev_rate = p.rate;
    }
}

static void test_negative_target(void)
{
    roll_profile_t p;
    roll_profile_init(&p, -TARGET, MAX_RATE, MAX_ACC);
    int ticks = 0;
    while (!p.done && ticks < (int)(CONTROL_LOOP_HZ * 10.0f)) { roll_profile_step(&p, DT); ticks++; }
    assert(p.done && fabsf(p.theta + TARGET) < 1e-3f);
}

int main(void)
{
    test_reaches_target_in_analytic_time();
    test_limits_respected_and_monotonic();
    test_negative_target();
    printf("roll_profile: OK\n");
    return 0;
}
