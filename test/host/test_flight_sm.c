// Drives the flight state machine through a synthetic flight profile and
// checks every transition and the post-apogee logging cutoff.
#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "control/flight_sm.h"

/* Published baro intervals the driver actually produces: the ~16.7 Hz cadence
 * the thresholds were first set against, and the ~94 Hz one a 10 ms poll gives
 * (one publish per poll bar one cycle in MS5611_TEMP_EVERY_N). The profile is
 * expressed in seconds and metres per second so both run the same flight, and
 * only the number of samples it is sliced into differs. */
#define BARO_DT_SLOW 0.06f
#define BARO_DT_FAST 0.0106f

/* Step the SM n ticks with constant inputs; returns last transition flag. */
static bool step_n(flight_sm_t *sm, int n, float az, float alt, float baro_alt,
                   bool baro_fresh, float dt)
{
    bool t = false;
    for (int i = 0; i < n; i++) {
        flight_sm_input_t in = {
            .az_mps2 = az, .alt_m = alt, .baro_alt_m = baro_alt,
            .baro_fresh = baro_fresh, .baro_dt_s = dt,
        };
        t = flight_sm_step(sm, &in);
    }
    return t;
}

/* Fly a constant vertical rate for secs, one baro publish per tick, and return
 * the altitude reached. Both alt_m and baro_alt_m follow the ramp. */
static float ramp(flight_sm_t *sm, float alt, float rate_mps, float secs, float dt)
{
    const int n = (int)(secs / dt);
    for (int i = 0; i < n; i++) {
        alt += rate_mps * dt;
        step_n(sm, 1, 0.0f, alt, alt, true, dt);
    }
    return alt;
}

static void test_full_flight(float dt)
{
    flight_sm_config_t cfg = flight_sm_config_default();
    flight_sm_t sm;
    flight_sm_init(&sm, &cfg);
    assert(sm.state == FLIGHT_STATE_STANDBY);

    /* Launch: az > 1.5 g must persist launch_debounce ticks. */
    step_n(&sm, cfg.launch_debounce - 1, 20.0f, 0, 0, false, dt);
    assert(sm.state == FLIGHT_STATE_STANDBY);
    step_n(&sm, 1, 20.0f, 0, 0, false, dt);
    assert(sm.state == FLIGHT_STATE_ACCELERATION);

    /* A single low-az tick must not trigger burnout (debounce resets). */
    step_n(&sm, 1, 0.5f, 100, 100, false, dt);
    step_n(&sm, 1, 20.0f, 110, 110, false, dt);
    step_n(&sm, cfg.burnout_debounce - 1, 0.5f, 200, 200, false, dt);
    assert(sm.state == FLIGHT_STATE_ACCELERATION);
    /* The aggregator keys the canard unlock off the transition flag. */
    assert(step_n(&sm, 1, 0.5f, 200, 200, false, dt));
    assert(sm.state == FLIGHT_STATE_FREE_FLIGHT);
    assert(!step_n(&sm, 1, 0.5f, 200, 200, false, dt));
    assert(sm.peak_alt >= 200.0f - 1e-3f);

    /* Coast up at +33 m/s for 6 s - stays in FREE_FLIGHT. */
    float alt = ramp(&sm, 200.0f, 33.3f, 6.0f, dt);
    assert(sm.state == FLIGHT_STATE_FREE_FLIGHT);

    /* Descend: baro velocity < -1 m/s AND >7.5 m below peak -> FALLING. */
    alt = ramp(&sm, alt, -33.3f, 0.7f, dt);
    assert(sm.state == FLIGHT_STATE_FALLING);

    /* Chute: baro velocity recovering above -5 m/s for recovery_debounce. */
    ramp(&sm, alt, 0.0f, 4.0f, dt);
    assert(sm.state == FLIGHT_STATE_RECOVERY);

    /* Mission completes post_falling_timeout_ticks after FALLING entry. */
    assert(!sm.mission_complete);
    for (uint32_t i = 0; i < cfg.post_falling_timeout_ticks; i++)
        step_n(&sm, 1, 0.0f, 100.0f, 100.0f, false, dt);
    assert(sm.mission_complete);
}

static void test_launch_debounce_resets(void)
{
    flight_sm_config_t cfg = flight_sm_config_default();
    flight_sm_t sm;
    flight_sm_init(&sm, &cfg);
    step_n(&sm, cfg.launch_debounce - 1, 20.0f, 0, 0, false, BARO_DT_SLOW);
    step_n(&sm, 1, 5.0f, 0, 0, false, BARO_DT_SLOW);   /* dip below threshold */
    step_n(&sm, cfg.launch_debounce - 1, 20.0f, 0, 0, false, BARO_DT_SLOW);
    assert(sm.state == FLIGHT_STATE_STANDBY);          /* counter restarted */
}

static void test_nan_baro_sample_is_ignored(float dt)
{
    flight_sm_config_t cfg = flight_sm_config_default();
    flight_sm_t sm;
    flight_sm_init(&sm, &cfg);

    /* Boost + burnout to reach FREE_FLIGHT, then coast up as in a real flight. */
    step_n(&sm, cfg.launch_debounce, 20.0f, 0, 0, false, dt);
    step_n(&sm, cfg.burnout_debounce, 0.5f, 200, 200, false, dt);
    assert(sm.state == FLIGHT_STATE_FREE_FLIGHT);
    float alt = ramp(&sm, 200.0f, 33.3f, 6.0f, dt);

    /* A NaN sample mid-coast must leave the velocity filter usable. */
    step_n(&sm, 1, 0.0f, alt, nanf(""), true, dt);
    assert(isfinite(sm.baro_vel_z));

    /* Apogee must still be detected afterwards. */
    ramp(&sm, alt, -33.3f, 0.7f, dt);
    assert(sm.state == FLIGHT_STATE_FALLING);
}

/* The velocity estimate is a difference over a held-open span, so its noise is
 * set by baro_vel_min_span_s and not by how finely the publishes are sliced.
 * A profile flown at both cadences must reach the same speed. */
static void test_vel_is_cadence_independent(void)
{
    flight_sm_config_t cfg = flight_sm_config_default();
    float vel[2];
    const float dts[2] = {BARO_DT_SLOW, BARO_DT_FAST};

    for (int i = 0; i < 2; i++) {
        flight_sm_t sm;
        flight_sm_init(&sm, &cfg);
        step_n(&sm, cfg.launch_debounce, 20.0f, 0, 0, false, dts[i]);
        step_n(&sm, cfg.burnout_debounce, 0.5f, 200, 200, false, dts[i]);
        assert(sm.state == FLIGHT_STATE_FREE_FLIGHT);
        ramp(&sm, 200.0f, 25.0f, 6.0f, dts[i]);
        vel[i] = sm.baro_vel_z;
    }

    assert(fabsf(vel[0] - 25.0f) < 1.0f);
    assert(fabsf(vel[1] - 25.0f) < 1.0f);
}

int main(void)
{
    test_full_flight(BARO_DT_SLOW);
    test_full_flight(BARO_DT_FAST);
    test_launch_debounce_resets();
    test_nan_baro_sample_is_ignored(BARO_DT_SLOW);
    test_nan_baro_sample_is_ignored(BARO_DT_FAST);
    test_vel_is_cadence_independent();
    printf("flight_sm: OK\n");
    return 0;
}
