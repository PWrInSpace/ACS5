// Closed-loop tests of the roll sequencer and cascade against a toy plant:
//   p_dot = K_CANARD * delta - C_DAMP * p + bias
#include "test_assert.h"
#include <math.h>
#include <stdio.h>

#include "control/roll_ctrl.h"
#include "utils/control_rate.h"
#include "utils/mathx.h"

/* The plant integrates on the same dt the controller runs at. */
#define CTRL_HZ  ((int)CONTROL_LOOP_HZ)
#define DT       CONTROL_DT_S
#define K_CANARD 50.0f    /* rad/s^2 per rad of deflection */
#define C_DAMP   2.0f     /* 1/s aero damping */

/* bias: constant roll torque (rad/s^2) modelling an aero trim offset. */
typedef struct { float p, angle, bias; quat_t q; } plant_t;

static void plant_init(plant_t *pl, float p0)
{
    pl->p = p0; pl->angle = 0.0f; pl->bias = 0.0f; pl->q = quat_identity();
}

static void plant_step(plant_t *pl, float delta)
{
    pl->p     += (K_CANARD * delta - C_DAMP * pl->p + pl->bias) * DT;
    pl->angle += pl->p * DT;
    float s = sinf(pl->angle * 0.5f);
    pl->q = (quat_t){cosf(pl->angle * 0.5f), 0.0f, 0.0f, s};
}

static roll_ctrl_config_t test_cfg(void)
{
    roll_ctrl_config_t c = roll_ctrl_config_default();
    c.kp_rate = 0.08f;            /* stronger than flight default: toy plant */
    /* Pinned rather than inherited from the defaults: the time budgets and
     * angle windows below depend on them. */
    c.kp_angle                = 2.0f;
    c.profile_max_rate_rads   = 1.8326f;    /* 105 deg/s */
    c.profile_max_accel_rads2 = 3.6652f;    /* 210 deg/s^2 */
    return c;
}

static void run(roll_ctrl_t *rc, plant_t *pl, int ticks, bool enabled, float *last_delta)
{
    for (int i = 0; i < ticks; i++) {
        roll_ctrl_input_t in = {
            .q = pl->q, .q_valid = true,
            .roll_rate_rads = pl->p, .maneuver_enabled = enabled,
        };
        float d = roll_ctrl_step(rc, &in);
        CHECK_NEAR(d, 0.0f, rc->cfg.max_deflection_rad + 1e-6f);
        plant_step(pl, d);
        if (last_delta) *last_delta = d;
    }
}

static void test_full_sequence(void)
{
    roll_ctrl_config_t cfg = test_cfg();
    roll_ctrl_t rc;
    roll_ctrl_init(&rc, &cfg);
    plant_t pl;
    plant_init(&pl, 3.5f);        /* enters free flight spinning 200 deg/s */

    CHECK(rc.phase == ROLL_CTRL_IDLE);
    run(&rc, &pl, CTRL_HZ * 8, true, NULL);   /* 8 s: damp + rotate + settle */

    CHECK(rc.phase == ROLL_CTRL_HOLD);
    CHECK_NEAR(rc.theta_meas, PI_F / 2.0f, 0.09f);   /* within ~5 deg */
    CHECK(fabsf(pl.angle) > 1.0f);          /* absolute angle, includes DAMP */
    CHECK_NEAR(pl.p, 0.0f, 0.35f);          /* re-stabilized (< 20 deg/s) */
}

static void test_disable_returns_to_idle_neutral(void)
{
    roll_ctrl_config_t cfg = test_cfg();
    roll_ctrl_t rc;
    roll_ctrl_init(&rc, &cfg);
    plant_t pl;
    plant_init(&pl, 0.0f);

    run(&rc, &pl, CTRL_HZ, true, NULL);
    CHECK(rc.phase != ROLL_CTRL_IDLE);

    float d = 1.0f;
    run(&rc, &pl, 10, false, &d);         /* apogee: gate drops */
    CHECK(rc.phase == ROLL_CTRL_IDLE);
    CHECK(d == 0.0f);                    /* canards neutral */
}

/* rate_sp is log-only: it must follow the outer-loop command in every phase. */
static void test_rate_sp_tracks_the_outer_loop(void)
{
    roll_ctrl_config_t cfg = test_cfg();
    roll_ctrl_t rc;
    roll_ctrl_init(&rc, &cfg);
    plant_t pl;
    plant_init(&pl, 3.5f);

    bool seen_rotating = false;
    for (int i = 0; i < CTRL_HZ * 8; i++) {
        roll_ctrl_input_t in = { .q = pl.q, .q_valid = true,
                                 .roll_rate_rads = pl.p, .maneuver_enabled = true };
        plant_step(&pl, roll_ctrl_step(&rc, &in));

        CHECK_NEAR(rc.rate_sp, 0.0f, cfg.rate_limit_rads + 1e-6f);
        if (rc.phase == ROLL_CTRL_DAMP || rc.phase == ROLL_CTRL_ABORT)
            CHECK(rc.rate_sp == 0.0f);
        if (rc.phase == ROLL_CTRL_ROTATE && fabsf(rc.rate_sp) > 0.1f)
            seen_rotating = true;
    }
    CHECK(rc.phase == ROLL_CTRL_HOLD);
    CHECK(seen_rotating);

    float d = 1.0f;
    run(&rc, &pl, 1, false, &d);
    CHECK(rc.rate_sp == 0.0f);   /* disarm clears it with the rest of the state */
}

static void test_abort_on_invalid_estimate(void)
{
    roll_ctrl_config_t cfg = test_cfg();
    roll_ctrl_t rc;
    roll_ctrl_init(&rc, &cfg);
    plant_t pl;
    plant_init(&pl, 0.0f);

    run(&rc, &pl, CTRL_HZ * 2, true, NULL);   /* get past DAMP */
    for (unsigned i = 0; i < cfg.abort_debounce_ticks; i++) {
        roll_ctrl_input_t in = { .q = pl.q, .q_valid = false,
                                 .roll_rate_rads = pl.p, .maneuver_enabled = true };
        roll_ctrl_step(&rc, &in);
    }
    CHECK(rc.phase == ROLL_CTRL_ABORT);
}

/* Fault bursts shorter than the debounce never abort, and do not accumulate. */
static void test_transient_fault_does_not_abort(void)
{
    roll_ctrl_config_t cfg = test_cfg();
    roll_ctrl_t rc;
    roll_ctrl_init(&rc, &cfg);
    plant_t pl;
    plant_init(&pl, 0.0f);

    CHECK(cfg.abort_debounce_ticks > 1u);   /* bursts below are N-1 ticks long */

    run(&rc, &pl, CTRL_HZ * 2, true, NULL);
    CHECK(rc.phase != ROLL_CTRL_ABORT);

    for (int burst = 0; burst < 5; burst++) {
        for (unsigned i = 0; i < cfg.abort_debounce_ticks - 1u; i++) {
            roll_ctrl_input_t bad = { .q = pl.q, .q_valid = false,
                                      .roll_rate_rads = pl.p,
                                      .maneuver_enabled = true };
            plant_step(&pl, roll_ctrl_step(&rc, &bad));
            CHECK(rc.phase != ROLL_CTRL_ABORT);
        }
        /* One good tick clears the count. */
        roll_ctrl_input_t good = { .q = pl.q, .q_valid = true,
                                   .roll_rate_rads = pl.p,
                                   .maneuver_enabled = true };
        plant_step(&pl, roll_ctrl_step(&rc, &good));
        CHECK(rc.abort_count == 0);
    }
    CHECK(rc.phase != ROLL_CTRL_ABORT);
}

/* A rejected quaternion never reaches the angle state (NaN would be absorbing). */
static void test_transient_bad_quaternion_keeps_output_finite(void)
{
    roll_ctrl_config_t cfg = test_cfg();
    roll_ctrl_t rc;
    roll_ctrl_init(&rc, &cfg);
    plant_t pl;
    plant_init(&pl, 0.0f);

    run(&rc, &pl, CTRL_HZ, true, NULL);
    CHECK(rc.phase == ROLL_CTRL_ROTATE);

    const quat_t q_nan = {NAN, 0.0f, 0.0f, 0.0f};

    /* q_valid true: the finiteness check, not the estimator flag, must catch it. */
    roll_ctrl_input_t bad = { .q = q_nan, .q_valid = true,
                              .roll_rate_rads = pl.p, .maneuver_enabled = true };
    float d = roll_ctrl_step(&rc, &bad);
    CHECK(isfinite(d));
    CHECK(isfinite(rc.theta_meas));
    plant_step(&pl, d);

    /* One good tick already cleared abort_count, so the maneuver runs on. */
    for (int i = 0; i < CTRL_HZ; i++) {
        roll_ctrl_input_t in = { .q = pl.q, .q_valid = true,
                                 .roll_rate_rads = pl.p, .maneuver_enabled = true };
        float dd = roll_ctrl_step(&rc, &in);
        CHECK(isfinite(dd));
        CHECK(isfinite(rc.theta_meas));
        plant_step(&pl, dd);
    }
    CHECK(rc.phase != ROLL_CTRL_ABORT);

    /* A persistent bad quaternion still aborts. */
    for (unsigned i = 0; i < cfg.abort_debounce_ticks; i++) {
        roll_ctrl_input_t in = { .q = q_nan, .q_valid = true,
                                 .roll_rate_rads = pl.p, .maneuver_enabled = true };
        CHECK(isfinite(roll_ctrl_step(&rc, &in)));
    }
    CHECK(rc.phase == ROLL_CTRL_ABORT);
}

/* Lost attitude aborts the maneuver but keeps damping on the gyro; the
 * reference is the same plant with neutral canards. */
static void test_abort_damps_on_gyro_when_attitude_is_lost(void)
{
    roll_ctrl_config_t cfg = test_cfg();
    roll_ctrl_t rc;
    roll_ctrl_init(&rc, &cfg);

    plant_t controlled, coasting;
    plant_init(&controlled, 3.5f);
    plant_init(&coasting, 3.5f);

    /* 200 ticks: at 100 the controlled/coasting rate ratio is still above 0.5. */
    for (int i = 0; i < 200; i++) {
        roll_ctrl_input_t in = { .q = controlled.q, .q_valid = false,
                                 .roll_rate_rads = controlled.p,
                                 .maneuver_enabled = true };
        plant_step(&controlled, roll_ctrl_step(&rc, &in));
        plant_step(&coasting, 0.0f);
    }

    CHECK(rc.phase == ROLL_CTRL_ABORT);
    CHECK_NEAR(controlled.p, 0.0f, 0.5f * fabsf(coasting.p));
}

/* ABORT keeps the learned trim integrator but drops every angle-derived term. */
static void test_abort_keeps_the_trim_correction(void)
{
    roll_ctrl_config_t cfg = test_cfg();
    roll_ctrl_t rc;
    roll_ctrl_init(&rc, &cfg);
    plant_t pl;
    plant_init(&pl, 0.0f);
    pl.bias = 0.3f;   /* needs bias/K_CANARD = 0.006 rad of standing deflection */

    run(&rc, &pl, CTRL_HZ * 6, true, NULL);
    CHECK(rc.phase == ROLL_CTRL_HOLD);
    CHECK(fabsf(rc.rate_integrator) > 0.003f);   /* learned the trim */

    for (unsigned i = 0; i < cfg.abort_debounce_ticks; i++) {
        roll_ctrl_input_t in = { .q = pl.q, .q_valid = false,
                                 .roll_rate_rads = pl.p, .maneuver_enabled = true };
        plant_step(&pl, roll_ctrl_step(&rc, &in));
    }
    CHECK(rc.phase == ROLL_CTRL_ABORT);
    CHECK(rc.rate_sp == 0.0f);                   /* no angle command survives */

    /* Estimate kept rejected so the controller stays in ABORT. */
    for (int i = 0; i < CTRL_HZ * 2; i++) {
        roll_ctrl_input_t in = { .q = pl.q, .q_valid = false,
                                 .roll_rate_rads = pl.p, .maneuver_enabled = true };
        plant_step(&pl, roll_ctrl_step(&rc, &in));
    }
    CHECK(rc.phase == ROLL_CTRL_ABORT);
    CHECK(fabsf(rc.rate_integrator) > 0.003f);   /* correction still carried */
    CHECK_NEAR(pl.p, 0.0f, 0.05f);   /* still damped against the trim it kept */
}

/* abort_debounce_ticks = 0 behaves as 1: abort on the first faulted tick. */
static void test_zero_debounce_still_aborts(void)
{
    roll_ctrl_config_t cfg = test_cfg();
    cfg.abort_debounce_ticks = 0;
    roll_ctrl_t rc;
    roll_ctrl_init(&rc, &cfg);
    plant_t pl;
    plant_init(&pl, 0.0f);

    run(&rc, &pl, CTRL_HZ, true, NULL);
    CHECK(rc.phase != ROLL_CTRL_ABORT);

    roll_ctrl_input_t bad = { .q = pl.q, .q_valid = false,
                              .roll_rate_rads = pl.p, .maneuver_enabled = true };
    plant_step(&pl, roll_ctrl_step(&rc, &bad));
    CHECK(rc.phase == ROLL_CTRL_ABORT);
}

/* A bad gyro sample is replaced by the last good one, not by zero. */
static void test_bad_gyro_holds_last_sample(void)
{
    roll_ctrl_config_t cfg = test_cfg();
    roll_ctrl_t rc;
    roll_ctrl_init(&rc, &cfg);

    /* Constant and above settle_rate_rads: stays in DAMP. */
    const float p_steady = 3.0f;
    float d = 0.0f;
    for (int i = 0; i < CTRL_HZ; i++) {
        roll_ctrl_input_t in = { .q = quat_identity(), .q_valid = true,
                                 .roll_rate_rads = p_steady,
                                 .maneuver_enabled = true };
        d = roll_ctrl_step(&rc, &in);
    }
    CHECK(rc.phase == ROLL_CTRL_DAMP);

    roll_ctrl_input_t bad = { .q = quat_identity(), .q_valid = true,
                              .roll_rate_rads = NAN, .maneuver_enabled = true };
    float d_bad = roll_ctrl_step(&rc, &bad);
    CHECK(isfinite(d_bad));
    CHECK_NEAR(d_bad, d, 1e-6f);
}

/* Recovery from a bad-gyro run keeps the command bounded and opposing the roll. */
static void test_bad_gyro_recovery_stays_bounded(void)
{
    roll_ctrl_config_t cfg = test_cfg();
    roll_ctrl_t rc;
    roll_ctrl_init(&rc, &cfg);
    plant_t pl;
    plant_init(&pl, 3.0f);

    run(&rc, &pl, 100, true, NULL);
    CHECK(rc.phase == ROLL_CTRL_DAMP);
    CHECK(pl.p > cfg.settle_rate_rads);   /* stays in DAMP across the window */

    /* One tick short of the debounce: the fault ends without aborting. */
    for (unsigned i = 0; i < cfg.abort_debounce_ticks - 1u; i++) {
        roll_ctrl_input_t bad = { .q = pl.q, .q_valid = true,
                                  .roll_rate_rads = NAN,
                                  .maneuver_enabled = true };
        float d = roll_ctrl_step(&rc, &bad);
        CHECK(isfinite(d));
        CHECK_NEAR(d, 0.0f, cfg.max_deflection_rad + 1e-6f);
        CHECK(d < 0.0f);                  /* still opposing a positive roll */
        plant_step(&pl, d);
    }
    CHECK(rc.phase != ROLL_CTRL_ABORT);

    for (int i = 0; i < 100; i++) {
        roll_ctrl_input_t in = { .q = pl.q, .q_valid = true,
                                 .roll_rate_rads = pl.p, .maneuver_enabled = true };
        float d = roll_ctrl_step(&rc, &in);
        CHECK(isfinite(d));
        CHECK_NEAR(d, 0.0f, cfg.max_deflection_rad + 1e-6f);
        CHECK(d < 0.0f);
        plant_step(&pl, d);
    }
    CHECK(rc.phase == ROLL_CTRL_DAMP);
    /* Magnitude bound: integral action closes the last error by crossing zero. */
    CHECK_NEAR(pl.p, 0.0f, 0.35f);
}

static void test_abort_on_excess_rate(void)
{
    roll_ctrl_config_t cfg = test_cfg();
    roll_ctrl_t rc;
    roll_ctrl_init(&rc, &cfg);
    plant_t pl;
    plant_init(&pl, 0.0f);

    roll_ctrl_input_t in = { .q = pl.q, .q_valid = true,
                             .roll_rate_rads = cfg.abort_rate_rads + 1.0f,
                             .maneuver_enabled = true };
    for (unsigned i = 0; i < cfg.abort_debounce_ticks - 1u; i++) {
        roll_ctrl_step(&rc, &in);
        CHECK(rc.phase != ROLL_CTRL_ABORT);
    }
    roll_ctrl_step(&rc, &in);
    CHECK(rc.phase == ROLL_CTRL_ABORT);
}

/* With no control authority the profile still completes and HOLD is reached. */
static void test_no_authority_still_reaches_hold(void)
{
    roll_ctrl_config_t cfg = test_cfg();
    cfg.max_deflection_rad = 0.0f;        /* no authority: plant cannot rotate */
    roll_ctrl_t rc;
    roll_ctrl_init(&rc, &cfg);
    plant_t pl;
    plant_init(&pl, 0.0f);

    run(&rc, &pl, CTRL_HZ * 3, true, NULL);
    CHECK(rc.phase == ROLL_CTRL_HOLD);
}

/* Golden DAMP output with the default config, captured from this implementation. */
static void test_damp_regression_pin(void)
{
    roll_ctrl_config_t cfg = roll_ctrl_config_default();
    /* Golden samples were captured at 500 Hz, independent of CONTROL_LOOP_HZ. */
    const int pin_hz = 500;
    cfg.dt_s = 1.0f / (float)pin_hz;
    roll_ctrl_t rc;
    roll_ctrl_init(&rc, &cfg);

    /* Rate offset above settle_rate_rads (stays in DAMP) yet small enough that
     * the open-loop integrator stays clear of the deflection limit and slew. */
    static const struct { int tick; float delta; } expect[] = {
        {   8, -0.085304655f },
        {   9, -0.088229969f },
        {  25, -0.111765578f },
        {  50, -0.110591412f },
        { 125, -0.095590502f },
        { 500, -0.152698770f },
    };

    size_t next = 0;
    for (int k = 1; k <= pin_hz; k++) {
        roll_ctrl_input_t in = {
            .q = quat_identity(), .q_valid = true,
            .roll_rate_rads = 0.8f + 0.3f * sinf(0.05f * (float)k),
            .maneuver_enabled = true,
        };
        float d = roll_ctrl_step(&rc, &in);
        if (next < sizeof expect / sizeof expect[0] && expect[next].tick == k) {
            /* Tolerance covers libm sinf ulp differences. */
            CHECK_NEAR(d, expect[next].delta, 1e-5f);
            next++;
        }
    }
    CHECK(next == sizeof expect / sizeof expect[0]);
    CHECK(rc.phase == ROLL_CTRL_DAMP);
}

/* A constant trim torque must not hold the sequencer in DAMP past the settle gate. */
static void test_constant_disturbance_still_reaches_rotate(void)
{
    roll_ctrl_config_t cfg = test_cfg();
    roll_ctrl_t rc;
    roll_ctrl_init(&rc, &cfg);
    plant_t pl;
    plant_init(&pl, 0.0f);
    pl.bias = 3.0f;   /* proportional-only settles at 0.5 rad/s; gate is 0.175 */

    run(&rc, &pl, CTRL_HZ * 3, true, NULL);

    CHECK(rc.phase == ROLL_CTRL_ROTATE || rc.phase == ROLL_CTRL_HOLD);
}

/* The rate integrator is frozen at its DAMP value throughout ROTATE. */
static void test_integrator_frozen_during_rotate(void)
{
    roll_ctrl_config_t cfg = test_cfg();
    roll_ctrl_t rc;
    roll_ctrl_init(&rc, &cfg);
    plant_t pl;
    plant_init(&pl, 0.0f);
    pl.bias = 0.3f;               /* something for DAMP to actually learn */

    for (int i = 0; i < CTRL_HZ * 6 && rc.phase != ROLL_CTRL_ROTATE; i++)
        run(&rc, &pl, 1, true, NULL);
    CHECK(rc.phase == ROLL_CTRL_ROTATE);
    CHECK(fabsf(rc.rate_integrator) > 1e-4f);   /* DAMP learned the trim */

    float frozen = rc.rate_integrator;
    for (int i = 0; i < CTRL_HZ / 2 && rc.phase == ROLL_CTRL_ROTATE; i++)
        run(&rc, &pl, 1, true, NULL);

    CHECK(rc.rate_integrator == frozen);
}

/* ROTATE hands over to HOLD when the profile ends, even if the angle lags. */
static void test_rotate_hands_over_to_hold_even_when_short(void)
{
    roll_ctrl_config_t cfg = test_cfg();
    cfg.kp_rate = 0.01f;          /* too sluggish to track the profile closely */
    roll_ctrl_t rc;
    roll_ctrl_init(&rc, &cfg);
    plant_t pl;
    plant_init(&pl, 0.0f);

    run(&rc, &pl, CTRL_HZ * 8, true, NULL);

    CHECK(rc.phase == ROLL_CTRL_HOLD);
}

/* Deterministic xorshift noise in [-1, 1). */
static uint32_t g_noise = 0x1234567u;
static float noise_step(void)
{
    g_noise ^= g_noise << 13; g_noise ^= g_noise >> 17; g_noise ^= g_noise << 5;
    return (float)(g_noise >> 8) / 8388608.0f - 1.0f;
}

/* Gyro noise must not turn into continuous servo travel. */
static void test_command_does_not_chatter_on_gyro_noise(void)
{
    roll_ctrl_config_t cfg = test_cfg();
    roll_ctrl_t rc;
    roll_ctrl_init(&rc, &cfg);

    const int n = 2000;
    float prev = 0.0f, travel = 0.0f;
    for (int i = 0; i < n; i++) {
        roll_ctrl_input_t in = {
            .q = quat_identity(), .q_valid = true,
            /* Above settle_rate_rads: stays in DAMP. */
            .roll_rate_rads = 0.5f + 0.01f * noise_step(),
            .maneuver_enabled = true,
        };
        float d = roll_ctrl_step(&rc, &in);
        if (i > 0) travel += fabsf(d - prev);
        prev = d;
    }
    CHECK(travel / (float)(n - 1) < 1e-3f);
}

/* Command change per tick never exceeds deflection_slew_rads * dt_s. */
static void test_slew_limit_bounds_command_rate(void)
{
    roll_ctrl_config_t cfg = test_cfg();
    roll_ctrl_t rc;
    roll_ctrl_init(&rc, &cfg);

    const float max_step = cfg.deflection_slew_rads * cfg.dt_s;
    float prev = 0.0f;
    bool  ever_bound = false;

    for (int i = 0; i < 200; i++) {
        /* Large rate demand whose sign flips every 20 ticks. */
        float rate = ((i / 20) % 2 == 0) ? 8.0f : -8.0f;
        roll_ctrl_input_t in = { .q = quat_identity(), .q_valid = true,
                                 .roll_rate_rads = rate, .maneuver_enabled = true };
        float d = roll_ctrl_step(&rc, &in);
        CHECK_NEAR(d, prev, max_step + 1e-6f);
        if (fabsf(fabsf(d - prev) - max_step) < 1e-7f) ever_bound = true;
        prev = d;
    }
    CHECK(ever_bound);
}

/* HOLD cancels a constant trim at the angle through the rate integrator alone. */
static void test_hold_rejects_constant_trim(void)
{
    roll_ctrl_config_t cfg = test_cfg();
    roll_ctrl_t rc;
    roll_ctrl_init(&rc, &cfg);
    plant_t pl;
    plant_init(&pl, 0.0f);
    pl.bias = 0.3f;

    run(&rc, &pl, CTRL_HZ * 10, true, NULL);
    CHECK(rc.phase == ROLL_CTRL_HOLD);

    /* Within a quarter degree of the 90 deg target, with the bias still on. */
    CHECK_NEAR(rc.theta_meas, cfg.target_angle_rad, 0.0044f);
    /* And holding it, not drifting through it. */
    CHECK_NEAR(pl.p, 0.0f, 0.02f);
}

/* A data-fault abort recovers and resumes against the original q_ref. */
static void test_data_fault_abort_recovers_against_the_same_reference(void)
{
    roll_ctrl_config_t cfg = test_cfg();
    roll_ctrl_t rc;
    roll_ctrl_init(&rc, &cfg);
    plant_t pl;
    plant_init(&pl, 0.0f);

    for (int i = 0; i < CTRL_HZ * 6 && rc.phase != ROLL_CTRL_ROTATE; i++)
        run(&rc, &pl, 1, true, NULL);
    CHECK(rc.phase == ROLL_CTRL_ROTATE);
    quat_t ref = rc.q_ref;

    for (unsigned i = 0; i < cfg.abort_debounce_ticks; i++) {
        roll_ctrl_input_t bad = { .q = pl.q, .q_valid = false,
                                  .roll_rate_rads = pl.p, .maneuver_enabled = true };
        plant_step(&pl, roll_ctrl_step(&rc, &bad));
    }
    CHECK(rc.phase == ROLL_CTRL_ABORT);

    run(&rc, &pl, CTRL_HZ * 6, true, NULL);          /* estimator comes back */

    CHECK(rc.phase == ROLL_CTRL_HOLD);
    CHECK(rc.q_ref.w == ref.w && rc.q_ref.x == ref.x &&
           rc.q_ref.y == ref.y && rc.q_ref.z == ref.z);
    CHECK_NEAR(rc.theta_meas, cfg.target_angle_rad, 0.09f);
}

/* An over-rate abort stays latched even after the data turns healthy. */
static void test_excess_rate_abort_stays_latched(void)
{
    roll_ctrl_config_t cfg = test_cfg();
    roll_ctrl_t rc;
    roll_ctrl_init(&rc, &cfg);
    plant_t pl;
    plant_init(&pl, 0.0f);

    run(&rc, &pl, CTRL_HZ * 2, true, NULL);

    for (unsigned i = 0; i < cfg.abort_debounce_ticks; i++) {
        roll_ctrl_input_t fast = { .q = pl.q, .q_valid = true,
                                   .roll_rate_rads = cfg.abort_rate_rads * 1.1f,
                                   .maneuver_enabled = true };
        roll_ctrl_step(&rc, &fast);
    }
    CHECK(rc.phase == ROLL_CTRL_ABORT);

    plant_init(&pl, 0.0f);                 /* perfectly healthy from here on */
    run(&rc, &pl, CTRL_HZ * 6, true, NULL);
    CHECK(rc.phase == ROLL_CTRL_ABORT);
}

/* Damping command after n ticks at qbar_pa from a fresh controller. */
static float damp_command_at(float qbar_pa, int ticks)
{
    roll_ctrl_config_t cfg = test_cfg();
    roll_ctrl_t rc;
    roll_ctrl_init(&rc, &cfg);

    float d = 0.0f;
    for (int i = 0; i < ticks; i++) {
        /* Small rate keeps the higher-gain run below the slew limit. */
        roll_ctrl_input_t in = { .q = quat_identity(), .q_valid = true,
                                 .roll_rate_rads = 0.1f, .qbar_pa = qbar_pa,
                                 .maneuver_enabled = true };
        d = roll_ctrl_step(&rc, &in);
    }
    return d;
}

/* Controller gain scales with qbar_ref / qbar. */
static void test_gain_scales_inversely_with_dynamic_pressure(void)
{
    roll_ctrl_config_t cfg = test_cfg();

    /* Two ticks: past the first low-pass sample, before the integrator matters. */
    float at_ref     = damp_command_at(cfg.qbar_ref_pa, 2);
    float at_quarter = damp_command_at(cfg.qbar_ref_pa * 0.25f, 2);

    CHECK(fabsf(at_ref) > 1e-4f);
    CHECK_NEAR(at_quarter / at_ref, 4.0f, 0.05f);
}

/* gain_scale stays within [gain_scale_min, gain_scale_max]. */
static void test_gain_scale_is_clamped(void)
{
    roll_ctrl_config_t cfg = test_cfg();
    roll_ctrl_t rc;
    roll_ctrl_init(&rc, &cfg);

    /* Just above the floor: scheduling active, unclamped ratio far above max. */
    roll_ctrl_input_t in = { .q = quat_identity(), .q_valid = true,
                             .roll_rate_rads = 0.4f,
                             .qbar_pa = cfg.qbar_floor_pa * 1.01f,
                             .maneuver_enabled = true };
    roll_ctrl_step(&rc, &in);
    CHECK(rc.gain_scale <= cfg.gain_scale_max + 1e-6f);
    CHECK(rc.gain_scale >= cfg.gain_scale_min - 1e-6f);
}

/* Below the qbar floor the command slews to the held integrator alone. */
static void test_below_qbar_floor_holds_the_trim_correction(void)
{
    roll_ctrl_config_t cfg = test_cfg();
    roll_ctrl_t rc;
    roll_ctrl_init(&rc, &cfg);

    for (int i = 0; i < CTRL_HZ; i++) {
        roll_ctrl_input_t in = { .q = quat_identity(), .q_valid = true,
                                 .roll_rate_rads = 0.4f,
                                 .qbar_pa = cfg.qbar_ref_pa,
                                 .maneuver_enabled = true };
        roll_ctrl_step(&rc, &in);
    }

    float prev = 1.0f, d = 0.0f;
    for (int i = 0; i < CTRL_HZ; i++) {
        roll_ctrl_input_t in = { .q = quat_identity(), .q_valid = true,
                                 .roll_rate_rads = 0.4f,
                                 .qbar_pa = cfg.qbar_floor_pa * 0.5f,
                                 .maneuver_enabled = true };
        d = roll_ctrl_step(&rc, &in);
        CHECK(fabsf(d - prev) <= cfg.deflection_slew_rads * cfg.dt_s + 1e-6f
               || i == 0);
        prev = d;
    }
    CHECK(rc.below_qbar_floor);
    CHECK_NEAR(d, rc.rate_integrator, 1e-4f);
    CHECK(fabsf(rc.rate_integrator) > 1e-3f);
}

/* qbar_pa = 0 means unknown: unity gain, floor not applied, controller active. */
static void test_absent_dynamic_pressure_disables_scheduling(void)
{
    roll_ctrl_config_t cfg = test_cfg();
    roll_ctrl_t rc;
    roll_ctrl_init(&rc, &cfg);

    roll_ctrl_input_t in = { .q = quat_identity(), .q_valid = true,
                             .roll_rate_rads = 0.4f, .qbar_pa = 0.0f,
                             .maneuver_enabled = true };
    float d = roll_ctrl_step(&rc, &in);

    CHECK(!rc.below_qbar_floor);
    CHECK(rc.gain_scale == 1.0f);
    CHECK(fabsf(d) > 1e-5f);
}

/* The integrator gain is scheduled with qbar like the proportional gain. */
static void test_integrator_is_scheduled_with_the_proportional_term(void)
{
    roll_ctrl_config_t cfg = test_cfg();

    float wound[2];
    const float qbar[2] = { cfg.qbar_ref_pa, cfg.qbar_ref_pa * 0.25f };
    for (int k = 0; k < 2; k++) {
        roll_ctrl_t rc;
        roll_ctrl_init(&rc, &cfg);
        for (int i = 0; i < CTRL_HZ / 4; i++) {
            roll_ctrl_input_t in = { .q = quat_identity(), .q_valid = true,
                                     .roll_rate_rads = 0.05f, .qbar_pa = qbar[k],
                                     .maneuver_enabled = true };
            roll_ctrl_step(&rc, &in);
        }
        wound[k] = rc.rate_integrator;
    }

    CHECK(fabsf(wound[0]) > 1e-5f);
    CHECK_NEAR(wound[1] / wound[0], 4.0f, 0.05f);
}

/* Over-rate latches the abort even when the tripping tick is an invalid sample. */
static void test_overspeed_latches_even_when_the_run_ends_invalid(void)
{
    roll_ctrl_config_t cfg = test_cfg();
    roll_ctrl_t rc;
    roll_ctrl_init(&rc, &cfg);
    plant_t pl;
    plant_init(&pl, 0.0f);

    run(&rc, &pl, CTRL_HZ * 2, true, NULL);

    for (unsigned i = 0; i < cfg.abort_debounce_ticks - 1u; i++) {
        roll_ctrl_input_t fast = { .q = pl.q, .q_valid = true,
                                   .roll_rate_rads = cfg.abort_rate_rads * 5.0f,
                                   .maneuver_enabled = true };
        roll_ctrl_step(&rc, &fast);
    }
    /* Gyro saturates to an invalid sample on the tripping tick. */
    roll_ctrl_input_t bad = { .q = pl.q, .q_valid = true, .roll_rate_rads = NAN,
                              .maneuver_enabled = true };
    roll_ctrl_step(&rc, &bad);
    CHECK(rc.phase == ROLL_CTRL_ABORT);

    plant_init(&pl, 0.0f);
    run(&rc, &pl, CTRL_HZ * 6, true, NULL);
    CHECK(rc.phase == ROLL_CTRL_ABORT);
}

/* Intermittent faults do not cycle the sequencer in and out of ABORT. */
static void test_intermittent_faults_do_not_thrash_the_sequencer(void)
{
    roll_ctrl_config_t cfg = test_cfg();
    roll_ctrl_t rc;
    roll_ctrl_init(&rc, &cfg);
    plant_t pl;
    plant_init(&pl, 0.0f);

    run(&rc, &pl, CTRL_HZ * 2, true, NULL);

    int aborts = 0;
    for (int i = 0; i < CTRL_HZ * 7; i++) {
        bool healthy = ((i / (int)cfg.abort_debounce_ticks) % 2) == 0;
        roll_ctrl_input_t in = { .q = pl.q, .q_valid = healthy,
                                 .roll_rate_rads = pl.p, .maneuver_enabled = true };
        roll_ctrl_phase_t prev = rc.phase;
        plant_step(&pl, roll_ctrl_step(&rc, &in));
        if (prev != ROLL_CTRL_ABORT && rc.phase == ROLL_CTRL_ABORT) aborts++;
    }
    CHECK(aborts <= 2);
}

/* An IMU dropout freezes the rate at a finite value; rate_stale must abort within the debounce. */
static void test_stale_rate_aborts_within_debounce(void)
{
    roll_ctrl_config_t cfg = test_cfg();
    roll_ctrl_t rc;
    roll_ctrl_init(&rc, &cfg);
    plant_t pl;
    plant_init(&pl, 0.0f);
    while (rc.phase != ROLL_CTRL_ROTATE) run(&rc, &pl, 1, true, NULL);
    run(&rc, &pl, CTRL_HZ / 4, true, NULL);

    const float frozen = pl.p;
    for (unsigned i = 0; i < cfg.abort_debounce_ticks; i++) {
        roll_ctrl_input_t in = { .q = pl.q, .q_valid = true, .roll_rate_rads = frozen,
                                 .rate_stale = true, .maneuver_enabled = true };
        plant_step(&pl, roll_ctrl_step(&rc, &in));
    }
    CHECK(rc.phase == ROLL_CTRL_ABORT);
}

/* In ABORT with no fresh rate the canards return to neutral and the integrator is left alone. */
static void test_abort_without_fresh_rate_goes_neutral(void)
{
    roll_ctrl_config_t cfg = test_cfg();
    roll_ctrl_t rc;
    roll_ctrl_init(&rc, &cfg);
    plant_t pl;
    plant_init(&pl, 3.0f);
    run(&rc, &pl, 100, true, NULL);

    float d = 0.0f;
    float integ = 0.0f;
    for (int i = 0; i < CTRL_HZ; i++) {
        roll_ctrl_input_t in = { .q = pl.q, .q_valid = true, .roll_rate_rads = 3.0f,
                                 .rate_stale = true, .maneuver_enabled = true };
        d = roll_ctrl_step(&rc, &in);
        if (rc.phase == ROLL_CTRL_ABORT && integ == 0.0f) integ = rc.rate_integrator;
    }
    CHECK(rc.phase == ROLL_CTRL_ABORT);
    CHECK_NEAR(d, 0.0f, 1e-6f);
    CHECK_NEAR(rc.rate_integrator, integ, 1e-6f);
}

int main(void)
{
    test_full_sequence();
    test_command_does_not_chatter_on_gyro_noise();
    test_integrator_is_scheduled_with_the_proportional_term();
    test_overspeed_latches_even_when_the_run_ends_invalid();
    test_intermittent_faults_do_not_thrash_the_sequencer();
    test_gain_scales_inversely_with_dynamic_pressure();
    test_gain_scale_is_clamped();
    test_below_qbar_floor_holds_the_trim_correction();
    test_absent_dynamic_pressure_disables_scheduling();
    test_data_fault_abort_recovers_against_the_same_reference();
    test_excess_rate_abort_stays_latched();
    test_hold_rejects_constant_trim();
    test_slew_limit_bounds_command_rate();
    test_constant_disturbance_still_reaches_rotate();
    test_integrator_frozen_during_rotate();
    test_rotate_hands_over_to_hold_even_when_short();
    test_disable_returns_to_idle_neutral();
    test_rate_sp_tracks_the_outer_loop();
    test_abort_on_invalid_estimate();
    test_transient_fault_does_not_abort();
    test_transient_bad_quaternion_keeps_output_finite();
    test_abort_damps_on_gyro_when_attitude_is_lost();
    test_abort_keeps_the_trim_correction();
    test_zero_debounce_still_aborts();
    test_bad_gyro_holds_last_sample();
    test_bad_gyro_recovery_stays_bounded();
    test_stale_rate_aborts_within_debounce();
    test_abort_without_fresh_rate_goes_neutral();
    test_abort_on_excess_rate();
    test_no_authority_still_reaches_hold();
    test_damp_regression_pin();
    return test_report("roll_ctrl");
}
