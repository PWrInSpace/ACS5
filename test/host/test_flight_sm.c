// Drives the flight state machine through a synthetic flight profile and
// checks every transition and the post-apogee logging cutoff.
#include "test_assert.h"
#include <math.h>
#include <stdio.h>

#include "flight/flight_sm.h"
#include "utils/control_rate.h"
#include "utils/mathx.h"

/* Baro publish intervals the driver can produce: ~16.7 Hz and ~94 Hz (10 ms poll,
 * minus one temperature cycle in MS5611_TEMP_EVERY_N). */
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
    CHECK(sm.state == FLIGHT_STATE_STANDBY);

    /* Launch: az > 1.5 g must persist launch_debounce ticks. */
    step_n(&sm, cfg.launch_debounce - 1, 20.0f, 0, 0, false, dt);
    CHECK(sm.state == FLIGHT_STATE_STANDBY);
    step_n(&sm, 1, 20.0f, 0, 0, false, dt);
    CHECK(sm.state == FLIGHT_STATE_ACCELERATION);

    /* A single low-az tick must not trigger burnout (debounce resets). */
    step_n(&sm, 1, 0.5f, 100, 100, false, dt);
    step_n(&sm, 1, 20.0f, 110, 110, false, dt);
    step_n(&sm, cfg.burnout_debounce - 1, 0.5f, 200, 200, false, dt);
    CHECK(sm.state == FLIGHT_STATE_ACCELERATION);
    /* The aggregator keys the canard unlock off the transition flag. */
    CHECK(step_n(&sm, 1, 0.5f, 200, 200, false, dt));
    CHECK(sm.state == FLIGHT_STATE_FREE_FLIGHT);
    CHECK(!step_n(&sm, 1, 0.5f, 200, 200, false, dt));
    CHECK(sm.peak_alt >= 200.0f - 1e-3f);

    /* Coast up at +33 m/s for 6 s - stays in FREE_FLIGHT. */
    float alt = ramp(&sm, 200.0f, 33.3f, 6.0f, dt);
    CHECK(sm.state == FLIGHT_STATE_FREE_FLIGHT);

    /* Descend: baro velocity < -1 m/s AND >7.5 m below peak -> FALLING. */
    alt = ramp(&sm, alt, -33.3f, 0.7f, dt);
    CHECK(sm.state == FLIGHT_STATE_FALLING);

    /* Chute: baro velocity recovering above -5 m/s for recovery_debounce. */
    ramp(&sm, alt, 0.0f, 4.0f, dt);
    CHECK(sm.state == FLIGHT_STATE_RECOVERY);

    /* Mission completes post_falling_timeout_ticks after FALLING entry. */
    CHECK(!sm.mission_complete);
    for (uint32_t i = 0; i < cfg.post_falling_timeout_ticks; i++)
        step_n(&sm, 1, 0.0f, 100.0f, 100.0f, false, dt);
    CHECK(sm.mission_complete);
}

static void test_launch_debounce_resets(void)
{
    flight_sm_config_t cfg = flight_sm_config_default();
    flight_sm_t sm;
    flight_sm_init(&sm, &cfg);
    step_n(&sm, cfg.launch_debounce - 1, 20.0f, 0, 0, false, BARO_DT_SLOW);
    step_n(&sm, 1, 5.0f, 0, 0, false, BARO_DT_SLOW);   /* dip below threshold */
    step_n(&sm, cfg.launch_debounce - 1, 20.0f, 0, 0, false, BARO_DT_SLOW);
    CHECK(sm.state == FLIGHT_STATE_STANDBY);          /* counter restarted */
}

static void test_nan_baro_sample_is_ignored(float dt)
{
    flight_sm_config_t cfg = flight_sm_config_default();
    flight_sm_t sm;
    flight_sm_init(&sm, &cfg);

    /* Boost + burnout to reach FREE_FLIGHT, then coast up as in a real flight. */
    step_n(&sm, cfg.launch_debounce, 20.0f, 0, 0, false, dt);
    step_n(&sm, cfg.burnout_debounce, 0.5f, 200, 200, false, dt);
    CHECK(sm.state == FLIGHT_STATE_FREE_FLIGHT);
    float alt = ramp(&sm, 200.0f, 33.3f, 6.0f, dt);

    /* A NaN sample mid-coast must leave the velocity filter usable. */
    step_n(&sm, 1, 0.0f, alt, nanf(""), true, dt);
    CHECK(isfinite(sm.baro_vel_z));

    /* Apogee must still be detected afterwards. */
    ramp(&sm, alt, -33.3f, 0.7f, dt);
    CHECK(sm.state == FLIGHT_STATE_FALLING);
}

/* Baro velocity is independent of the publish cadence. */
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
        CHECK(sm.state == FLIGHT_STATE_FREE_FLIGHT);
        ramp(&sm, 200.0f, 25.0f, 6.0f, dts[i]);
        vel[i] = sm.baro_vel_z;
    }

    CHECK_NEAR(vel[0], 25.0f, 1.0f);
    CHECK_NEAR(vel[1], 25.0f, 1.0f);
}

/* One control tick with a fresh baro sample. */
static void tick_baro(flight_sm_t *sm, float az, float baro_alt)
{
    flight_sm_input_t in = { .az_mps2 = az, .alt_m = baro_alt, .baro_alt_m = baro_alt,
                             .baro_fresh = true, .baro_dt_s = CONTROL_DT_S };
    flight_sm_step(sm, &in);
}

/* A jolt on the pad that trips launch detection is undone once baro and IMU show no climb. */
static void test_false_launch_reverts_to_standby(void)
{
    flight_sm_config_t cfg = flight_sm_config_default();
    flight_sm_t sm;
    flight_sm_init(&sm, &cfg);
    for (int i = 0; i < 100; i++) tick_baro(&sm, GRAVITY, 0.0f);

    for (int i = 0; i < cfg.launch_debounce; i++) tick_baro(&sm, 2.0f * GRAVITY, 0.0f);
    CHECK(sm.state == FLIGHT_STATE_ACCELERATION);
    for (uint32_t i = 0; i < cfg.launch_confirm_ticks; i++) tick_baro(&sm, GRAVITY, 0.1f);
    CHECK(sm.state == FLIGHT_STATE_STANDBY);
    CHECK(sm.false_launch_count == 1u);

    for (int i = 0; i < cfg.launch_debounce; i++) tick_baro(&sm, 5.0f * GRAVITY, 0.0f);
    CHECK(sm.state == FLIGHT_STATE_ACCELERATION);
}

/* A weak motor (thrust 1.6 g) climbs only metres in the window; the IMU still confirms it. */
static void test_weak_real_launch_is_kept(void)
{
    flight_sm_config_t cfg = flight_sm_config_default();
    flight_sm_t sm;
    flight_sm_init(&sm, &cfg);
    for (int i = 0; i < 100; i++) tick_baro(&sm, GRAVITY, 0.0f);

    const float az = 1.6f * GRAVITY;
    float v = 0.0f, h = 0.0f;
    for (uint32_t i = 0; i < 3u * cfg.launch_confirm_ticks; i++) {
        v += (az - GRAVITY) * CONTROL_DT_S;
        h += v * CONTROL_DT_S;
        tick_baro(&sm, az, h);
    }
    CHECK(sm.state == FLIGHT_STATE_ACCELERATION);
    CHECK(sm.false_launch_count == 0u);
}

/* Without baro data there is nothing to confirm against; the launch stands. */
static void test_launch_without_baro_is_kept(void)
{
    flight_sm_config_t cfg = flight_sm_config_default();
    flight_sm_t sm;
    flight_sm_init(&sm, &cfg);
    step_n(&sm, cfg.launch_debounce, 2.0f * GRAVITY, 0, 0, false, CONTROL_DT_S);
    step_n(&sm, (int)cfg.launch_confirm_ticks + 10, GRAVITY, 0, 0, false, CONTROL_DT_S);
    CHECK(sm.state == FLIGHT_STATE_ACCELERATION);
}

/* A dead barometer cannot hold FREE_FLIGHT (and so the armed controller) past max_coast. */
static void test_coast_timeout_reaches_falling(void)
{
    flight_sm_config_t cfg = flight_sm_config_default();
    flight_sm_t sm;
    flight_sm_init(&sm, &cfg);
    step_n(&sm, cfg.launch_debounce, 20.0f, 0, 0, false, CONTROL_DT_S);
    step_n(&sm, cfg.burnout_debounce, 0.5f, 200, 200, false, CONTROL_DT_S);
    CHECK(sm.state == FLIGHT_STATE_FREE_FLIGHT);

    step_n(&sm, (int)cfg.max_coast_ticks - 1, 0.0f, 300, 300, false, CONTROL_DT_S);
    CHECK(sm.state == FLIGHT_STATE_FREE_FLIGHT);
    step_n(&sm, 1, 0.0f, 300, 300, false, CONTROL_DT_S);
    CHECK(sm.state == FLIGHT_STATE_FALLING);
}

/* The baro velocity filter starts from the estimator's speed at burnout, not from zero. */
static void test_burnout_seeds_baro_velocity(void)
{
    flight_sm_config_t cfg = flight_sm_config_default();
    flight_sm_t sm;
    flight_sm_init(&sm, &cfg);
    step_n(&sm, cfg.launch_debounce, 20.0f, 0, 0, false, CONTROL_DT_S);
    for (int i = 0; i < cfg.burnout_debounce; i++) {
        flight_sm_input_t in = { .az_mps2 = 0.5f, .alt_m = 150.0f, .baro_alt_m = 150.0f,
                                 .vel_z_mps = 77.6f };
        flight_sm_step(&sm, &in);
    }
    CHECK(sm.state == FLIGHT_STATE_FREE_FLIGHT);
    CHECK_NEAR(sm.baro_vel_z, 77.6f, 1e-3f);
}

int main(void)
{
    test_full_flight(BARO_DT_SLOW);
    test_full_flight(BARO_DT_FAST);
    test_launch_debounce_resets();
    test_nan_baro_sample_is_ignored(BARO_DT_SLOW);
    test_nan_baro_sample_is_ignored(BARO_DT_FAST);
    test_vel_is_cadence_independent();
    test_false_launch_reverts_to_standby();
    test_weak_real_launch_is_kept();
    test_launch_without_baro_is_kept();
    test_coast_timeout_reaches_falling();
    test_burnout_seeds_baro_velocity();
    return test_report("flight_sm");
}
