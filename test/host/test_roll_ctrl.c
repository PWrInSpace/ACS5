// Closed-loop test of the sequencer + cascade against a toy roll plant:
//   p_dot = K_CANARD * delta - C_DAMP * p
// Numbers give a fast, well-damped inner loop so the test checks sequencing
// and convergence, not flight-gain quality (that is the simulator's job).
#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "control/roll_ctrl.h"
#include "control/control_rate.h"
#include "utils/mathx.h"

/* The plant integrates on the same dt the controller runs at. */
#define CTRL_HZ  ((int)CONTROL_LOOP_HZ)
#define DT       CONTROL_DT_S
#define K_CANARD 50.0f    /* rad/s^2 per rad of deflection */
#define C_DAMP   2.0f     /* 1/s aero damping */

/* bias is a constant roll torque (rad/s^2), standing in for an aero trim
 * offset; it forces a nonzero steady-state deflection in HOLD. */
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
    /* Pinned, not inherited: tuning retunes the production defaults, and every
     * one of these sets a margin the tests below depend on. kp_angle sets how
     * hard HOLD pulls the residual angle error back against the bias torque,
     * which is what test_full_sequence's 5 deg window measures. The profile
     * limits set how long the 90 deg rotation takes, which has to fit inside
     * that test's 8 s budget with room for the convergence it then asserts. */
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
        assert(fabsf(d) <= rc->cfg.max_deflection_rad + 1e-6f);
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

    assert(rc.phase == ROLL_CTRL_IDLE);
    run(&rc, &pl, CTRL_HZ * 8, true, NULL);   /* 8 s: damp + rotate + settle */

    assert(rc.phase == ROLL_CTRL_HOLD);
    assert(fabsf(rc.theta_meas - PI_F / 2.0f) < 0.09f);   /* within ~5 deg */
    /* Weak sanity check that the plant moved at all. This is absolute angle,
     * so it also carries whatever DAMP shed from p0 - theta_meas above is what
     * pins the 90 deg. */
    assert(fabsf(pl.angle) > 1.0f);
    assert(fabsf(pl.p) < 0.35f);          /* re-stabilized (< 20 deg/s) */
}

static void test_disable_returns_to_idle_neutral(void)
{
    roll_ctrl_config_t cfg = test_cfg();
    roll_ctrl_t rc;
    roll_ctrl_init(&rc, &cfg);
    plant_t pl;
    plant_init(&pl, 0.0f);

    run(&rc, &pl, CTRL_HZ, true, NULL);
    assert(rc.phase != ROLL_CTRL_IDLE);

    float d = 1.0f;
    run(&rc, &pl, 10, false, &d);         /* apogee: gate drops */
    assert(rc.phase == ROLL_CTRL_IDLE);
    assert(d == 0.0f);                    /* canards neutral */
}

/* rate_sp exists only to be logged - nothing in the loop reads it back - so
 * without this the field could silently stop tracking the outer loop and the
 * flight log would carry a plausible-looking lie into system identification. */
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

        assert(fabsf(rc.rate_sp) <= cfg.rate_limit_rads + 1e-6f);
        if (rc.phase == ROLL_CTRL_DAMP || rc.phase == ROLL_CTRL_ABORT)
            assert(rc.rate_sp == 0.0f);
        if (rc.phase == ROLL_CTRL_ROTATE && fabsf(rc.rate_sp) > 0.1f)
            seen_rotating = true;
    }
    assert(rc.phase == ROLL_CTRL_HOLD);
    assert(seen_rotating);

    float d = 1.0f;
    run(&rc, &pl, 1, false, &d);
    assert(rc.rate_sp == 0.0f);   /* disarm clears it with the rest of the state */
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
    assert(rc.phase == ROLL_CTRL_ABORT);
}

/* The debounce is the whole point of not latching on one sample: a blip
 * shorter than the window must leave the maneuver running, and repeated
 * blips must never accumulate across the good ticks between them. */
static void test_transient_fault_does_not_abort(void)
{
    roll_ctrl_config_t cfg = test_cfg();
    roll_ctrl_t rc;
    roll_ctrl_init(&rc, &cfg);
    plant_t pl;
    plant_init(&pl, 0.0f);

    /* At a threshold of 1 there is no debounce and the burst below would be
     * zero ticks long, so the test would pass without exercising anything. */
    assert(cfg.abort_debounce_ticks > 1u);

    run(&rc, &pl, CTRL_HZ * 2, true, NULL);
    assert(rc.phase != ROLL_CTRL_ABORT);

    for (int burst = 0; burst < 5; burst++) {
        for (unsigned i = 0; i < cfg.abort_debounce_ticks - 1u; i++) {
            roll_ctrl_input_t bad = { .q = pl.q, .q_valid = false,
                                      .roll_rate_rads = pl.p,
                                      .maneuver_enabled = true };
            plant_step(&pl, roll_ctrl_step(&rc, &bad));
            assert(rc.phase != ROLL_CTRL_ABORT);
        }
        /* One good tick must clear the count, or the bursts would sum. */
        roll_ctrl_input_t good = { .q = pl.q, .q_valid = true,
                                   .roll_rate_rads = pl.p,
                                   .maneuver_enabled = true };
        plant_step(&pl, roll_ctrl_step(&rc, &good));
        assert(rc.abort_count == 0);
    }
    assert(rc.phase != ROLL_CTRL_ABORT);
}

/* A rejected quaternion must never reach the angle state. The debounce leaves
 * the maneuver running for up to N-1 faulted ticks, and NaN is absorbing: it
 * propagates through quat_mul into theta_meas and theta_prev_wrapped, clampf
 * passes it through (both of its comparisons are false on NaN), and the inner
 * loop's low-pass and slew are self-referential, so a single bad sample would
 * pin the deflection at NaN for the rest of the armed window - long after the
 * data recovers and the abort count has reset. */
static void test_transient_bad_quaternion_keeps_output_finite(void)
{
    roll_ctrl_config_t cfg = test_cfg();
    roll_ctrl_t rc;
    roll_ctrl_init(&rc, &cfg);
    plant_t pl;
    plant_init(&pl, 0.0f);

    run(&rc, &pl, CTRL_HZ, true, NULL);
    assert(rc.phase == ROLL_CTRL_ROTATE);

    const quat_t q_nan = {NAN, 0.0f, 0.0f, 0.0f};

    /* q_valid true on purpose: the finiteness check, not the estimator flag,
     * is what has to catch this. */
    roll_ctrl_input_t bad = { .q = q_nan, .q_valid = true,
                              .roll_rate_rads = pl.p, .maneuver_enabled = true };
    float d = roll_ctrl_step(&rc, &bad);
    assert(isfinite(d));
    assert(isfinite(rc.theta_meas));
    plant_step(&pl, d);

    /* One good tick already cleared abort_count, so the maneuver runs on. */
    for (int i = 0; i < CTRL_HZ; i++) {
        roll_ctrl_input_t in = { .q = pl.q, .q_valid = true,
                                 .roll_rate_rads = pl.p, .maneuver_enabled = true };
        float dd = roll_ctrl_step(&rc, &in);
        assert(isfinite(dd));
        assert(isfinite(rc.theta_meas));
        plant_step(&pl, dd);
    }
    assert(rc.phase != ROLL_CTRL_ABORT);

    /* Skipping the angle update must not cost the latch when it persists. */
    for (unsigned i = 0; i < cfg.abort_debounce_ticks; i++) {
        roll_ctrl_input_t in = { .q = q_nan, .q_valid = true,
                                 .roll_rate_rads = pl.p, .maneuver_enabled = true };
        assert(isfinite(roll_ctrl_step(&rc, &in)));
    }
    assert(rc.phase == ROLL_CTRL_ABORT);
}

/* A dead attitude estimate ends the maneuver but must not disarm the
 * damper - the gyro is still good. Benchmark against the same plant with
 * neutral canards: that is what a controller blinded by q_valid would do. */
static void test_abort_damps_on_gyro_when_attitude_is_lost(void)
{
    roll_ctrl_config_t cfg = test_cfg();
    roll_ctrl_t rc;
    roll_ctrl_init(&rc, &cfg);

    plant_t controlled, coasting;
    plant_init(&controlled, 3.5f);
    plant_init(&coasting, 3.5f);

    /* 0.4 s: the horizon, not the latch tick, is what sets the margin below.
     * The damper adds K_CANARD*kp_rate to C_DAMP, so the controlled rate decays
     * at roughly 6/s against the coasting plant's 2/s and the ratio falls off
     * exponentially with the window: 0.58 at 100 ticks, which does not clear
     * the assert at all, against 0.33 at 200. The longer horizon buys margin
     * without costing sensitivity - at half the damper gain (kp_rate 0.04) the
     * ratio is 0.61 and this still fails. */
    for (int i = 0; i < 200; i++) {
        roll_ctrl_input_t in = { .q = controlled.q, .q_valid = false,
                                 .roll_rate_rads = controlled.p,
                                 .maneuver_enabled = true };
        plant_step(&controlled, roll_ctrl_step(&rc, &in));
        plant_step(&coasting, 0.0f);
    }

    assert(rc.phase == ROLL_CTRL_ABORT);
    assert(fabsf(controlled.p) < 0.5f * fabsf(coasting.p));
}

/* ABORT is pure rate damping, and on a trimmed airframe holding the rate at
 * zero *is* a standing deflection - the one the integrator learned. Clearing it
 * on the way in would hand ABORT back the steady rate error the trim produces,
 * which is the opposite of damping well. So the integrator carries over, and
 * what must not carry over is any angle-derived term: ABORT commands no angle. */
static void test_abort_keeps_the_trim_correction(void)
{
    roll_ctrl_config_t cfg = test_cfg();
    roll_ctrl_t rc;
    roll_ctrl_init(&rc, &cfg);
    plant_t pl;
    plant_init(&pl, 0.0f);
    pl.bias = 0.3f;   /* needs bias/K_CANARD = 0.006 rad of standing deflection */

    run(&rc, &pl, CTRL_HZ * 6, true, NULL);
    assert(rc.phase == ROLL_CTRL_HOLD);
    assert(fabsf(rc.rate_integrator) > 0.003f);   /* learned the trim */

    for (unsigned i = 0; i < cfg.abort_debounce_ticks; i++) {
        roll_ctrl_input_t in = { .q = pl.q, .q_valid = false,
                                 .roll_rate_rads = pl.p, .maneuver_enabled = true };
        plant_step(&pl, roll_ctrl_step(&rc, &in));
    }
    assert(rc.phase == ROLL_CTRL_ABORT);
    assert(rc.rate_sp == 0.0f);                   /* no angle command survives */

    /* Hold the estimate rejected so this measures ABORT and not the phase it
     * recovers into - the damping claim is about ABORT. */
    for (int i = 0; i < CTRL_HZ * 2; i++) {
        roll_ctrl_input_t in = { .q = pl.q, .q_valid = false,
                                 .roll_rate_rads = pl.p, .maneuver_enabled = true };
        plant_step(&pl, roll_ctrl_step(&rc, &in));
    }
    assert(rc.phase == ROLL_CTRL_ABORT);
    assert(fabsf(rc.rate_integrator) > 0.003f);   /* correction still carried */
    assert(fabsf(pl.p) < 0.05f);   /* still damped against the trim it kept */
}

/* 0 is the natural way to spell "no debounce", but it makes the threshold
 * comparison false before the counter can move. roll_ctrl_init clamps it to
 * exactly 1: asserting on the first faulted tick is what pins that value, where
 * a longer run of faulted ticks would pass for any clamp up to its length. */
static void test_zero_debounce_still_aborts(void)
{
    roll_ctrl_config_t cfg = test_cfg();
    cfg.abort_debounce_ticks = 0;
    roll_ctrl_t rc;
    roll_ctrl_init(&rc, &cfg);
    plant_t pl;
    plant_init(&pl, 0.0f);

    run(&rc, &pl, CTRL_HZ, true, NULL);
    assert(rc.phase != ROLL_CTRL_ABORT);

    roll_ctrl_input_t bad = { .q = pl.q, .q_valid = false,
                              .roll_rate_rads = pl.p, .maneuver_enabled = true };
    plant_step(&pl, roll_ctrl_step(&rc, &bad));
    assert(rc.phase == ROLL_CTRL_ABORT);
}

/* A bad gyro holds the last good sample rather than substituting zero. Zero
 * would claim the airframe had stopped rotating, and the loop would unwind the
 * damping command it is holding for as long as the fault lasted. Held, the loop
 * sees the previous sample; with the rate constant, as it is here, that is
 * exactly what a good tick would have given. */
static void test_bad_gyro_holds_last_sample(void)
{
    roll_ctrl_config_t cfg = test_cfg();
    roll_ctrl_t rc;
    roll_ctrl_init(&rc, &cfg);

    /* Well clear of settle_rate_rads, so this stays in DAMP and measures the
     * inner loop alone; constant, so it settles to a fixed deflection. */
    const float p_steady = 3.0f;
    float d = 0.0f;
    for (int i = 0; i < CTRL_HZ; i++) {
        roll_ctrl_input_t in = { .q = quat_identity(), .q_valid = true,
                                 .roll_rate_rads = p_steady,
                                 .maneuver_enabled = true };
        d = roll_ctrl_step(&rc, &in);
    }
    assert(rc.phase == ROLL_CTRL_DAMP);

    roll_ctrl_input_t bad = { .q = quat_identity(), .q_valid = true,
                              .roll_rate_rads = NAN, .maneuver_enabled = true };
    float d_bad = roll_ctrl_step(&rc, &bad);
    assert(isfinite(d_bad));
    assert(fabsf(d_bad - d) < 1e-6f);
}

/* The first good tick after a fault presents the whole rate change that
 * happened during it as one step in the error. The command must absorb that
 * without reversing: the low-pass admits only target_lpf_alpha of it and the
 * slew caps the move at deflection_slew_rads*dt_s, so the recovery costs a few
 * ticks of damping authority and nothing more. That bounded cost is what this
 * pins. */
static void test_bad_gyro_recovery_stays_bounded(void)
{
    roll_ctrl_config_t cfg = test_cfg();
    roll_ctrl_t rc;
    roll_ctrl_init(&rc, &cfg);
    plant_t pl;
    plant_init(&pl, 3.0f);

    run(&rc, &pl, 100, true, NULL);
    assert(rc.phase == ROLL_CTRL_DAMP);
    assert(pl.p > cfg.settle_rate_rads);   /* stays in DAMP across the window */

    /* One tick short of the debounce, so the fault ends without latching and
     * the recovery tick below is reached at all. */
    for (unsigned i = 0; i < cfg.abort_debounce_ticks - 1u; i++) {
        roll_ctrl_input_t bad = { .q = pl.q, .q_valid = true,
                                  .roll_rate_rads = NAN,
                                  .maneuver_enabled = true };
        float d = roll_ctrl_step(&rc, &bad);
        assert(isfinite(d));
        assert(fabsf(d) <= cfg.max_deflection_rad + 1e-6f);
        assert(d < 0.0f);                  /* still opposing a positive roll */
        plant_step(&pl, d);
    }
    assert(rc.phase != ROLL_CTRL_ABORT);

    for (int i = 0; i < 100; i++) {
        roll_ctrl_input_t in = { .q = pl.q, .q_valid = true,
                                 .roll_rate_rads = pl.p, .maneuver_enabled = true };
        float d = roll_ctrl_step(&rc, &in);
        assert(isfinite(d));
        assert(fabsf(d) <= cfg.max_deflection_rad + 1e-6f);
        assert(d < 0.0f);
        plant_step(&pl, d);
    }
    assert(rc.phase == ROLL_CTRL_DAMP);
    /* Down from 3 rad/s to under 20 deg/s in either direction. The bound is on
     * magnitude because integral action closes the last of the error by
     * crossing zero rather than creeping up to it; what must not happen is the
     * rate growing, or flipping into a spin the other way, which is what a sign
     * error or a runaway would look like here. */
    assert(fabsf(pl.p) < 0.35f);
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
        assert(rc.phase != ROLL_CTRL_ABORT);
    }
    roll_ctrl_step(&rc, &in);
    assert(rc.phase == ROLL_CTRL_ABORT);
}

/* A vehicle that cannot rotate at all is the worst case the old watchdog was
 * written for, and it used to end in ABORT - canards standing down for the rest
 * of the coast. It now ends in HOLD: the profile completes regardless of what
 * the airframe managed, and HOLD keeps the angle loop and the damper working
 * with whatever authority does exist. Nothing here is a sensor fault, so
 * nothing here justifies abandoning angle control. */
static void test_no_authority_still_reaches_hold(void)
{
    roll_ctrl_config_t cfg = test_cfg();
    cfg.max_deflection_rad = 0.0f;        /* no authority: plant cannot rotate */
    roll_ctrl_t rc;
    roll_ctrl_init(&rc, &cfg);
    plant_t pl;
    plant_init(&pl, 0.0f);

    run(&rc, &pl, CTRL_HZ * 3, true, NULL);
    assert(rc.phase == ROLL_CTRL_HOLD);
}

/* Regression pin on DAMP with the shipped defaults.
 *
 * The expected values below were captured from this implementation and have no
 * authority beyond that: they assert only that DAMP still does what it does
 * today. Tasks 6-8 rewire the call sites around this controller, and a diff
 * here means one of them changed the damper's numeric behavior - which may be
 * intended, but must not happen silently. */
static void test_damp_regression_pin(void)
{
    roll_ctrl_config_t cfg = roll_ctrl_config_default();
    /* Golden samples were captured at 500 Hz dt; pin that so a CONTROL_LOOP_HZ
     * flight-rate change does not silently invalidate this regression. */
    const int pin_hz = 500;
    cfg.dt_s = 1.0f / (float)pin_hz;
    roll_ctrl_t rc;
    roll_ctrl_init(&rc, &cfg);

    /* Offset well clear of settle_rate_rads so the sequencer stays in DAMP
     * for the whole run and this measures the inner loop alone.
     *
     * The offset also has to stay small enough to leave headroom under the
     * deflection limit. This scenario feeds a fixed rate with no plant behind
     * it, so the error never decays and the integrator keeps growing; a larger
     * offset parks the late rows within a few percent of the limit, where a
     * retune would clip instead of showing up as a diff.
     *
     * Every row is a joint function of kp_rate, ki_rate and target_lpf_alpha,
     * which is what makes them catch a retune. The command never reaches the
     * slew limit at this excitation, so nothing here constrains it -
     * test_slew_limit_bounds_command_rate covers that separately. The last row
     * is where the integrator dominates: by tick 500 it carries most of the
     * command. */
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
            /* Loose enough to absorb libm sinf differing by an ulp or two
             * across versions, tight enough that any real retune shows. */
            assert(fabsf(d - expect[next].delta) < 1e-5f);
            next++;
        }
    }
    assert(next == sizeof expect / sizeof expect[0]);
    assert(rc.phase == ROLL_CTRL_DAMP);
}

/* A steady aero trim torque leaves a proportional rate loop with a standing
 * error of bias/(K_CANARD*kp_rate + C_DAMP). The settle gate is what that error
 * runs into, so on the real airframe a quarter degree of fin misalignment - a
 * tolerance nobody guarantees - is already enough to hold the sequencer in DAMP
 * for the whole coast and skip the maneuver entirely. */
static void test_constant_disturbance_still_reaches_rotate(void)
{
    roll_ctrl_config_t cfg = test_cfg();
    roll_ctrl_t rc;
    roll_ctrl_init(&rc, &cfg);
    plant_t pl;
    plant_init(&pl, 0.0f);
    pl.bias = 3.0f;   /* proportional-only settles at 0.5 rad/s; gate is 0.175 */

    run(&rc, &pl, CTRL_HZ * 3, true, NULL);

    assert(rc.phase == ROLL_CTRL_ROTATE || rc.phase == ROLL_CTRL_HOLD);
}

/* The integrator cancels a standing trim torque. Tracking lag behind the moving
 * profile is not one: it is the loop doing its job, it reverses sign when the
 * profile decelerates, and integrating it hands ROTATE a wound-up bias that
 * carries straight through the target. The trim does not change over the two
 * seconds of the rotation, so freezing what DAMP learned keeps the correction
 * without the windup. */
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
    assert(rc.phase == ROLL_CTRL_ROTATE);
    assert(fabsf(rc.rate_integrator) > 1e-4f);   /* DAMP learned the trim */

    float frozen = rc.rate_integrator;
    for (int i = 0; i < CTRL_HZ / 2 && rc.phase == ROLL_CTRL_ROTATE; i++)
        run(&rc, &pl, 1, true, NULL);

    assert(rc.rate_integrator == frozen);
}

/* A rotation that ends a few degrees short is a maneuver that nearly worked,
 * and HOLD is the phase built to close that gap - it has the angle loop and the
 * integrator. Gating the handover on the error being small already meant the
 * near-miss sat in ROTATE until the watchdog degraded it to plain damping,
 * throwing away the whole maneuver to avoid finishing it imperfectly. */
static void test_rotate_hands_over_to_hold_even_when_short(void)
{
    roll_ctrl_config_t cfg = test_cfg();
    cfg.kp_rate = 0.01f;          /* too sluggish to track the profile closely */
    roll_ctrl_t rc;
    roll_ctrl_init(&rc, &cfg);
    plant_t pl;
    plant_init(&pl, 0.0f);

    run(&rc, &pl, CTRL_HZ * 8, true, NULL);

    assert(rc.phase == ROLL_CTRL_HOLD);
}

/* Deterministic, so the dither bound below means the same thing on every run
 * and every libm. Range [-1, 1). */
static uint32_t g_noise = 0x1234567u;
static float noise_step(void)
{
    g_noise ^= g_noise << 13; g_noise ^= g_noise >> 17; g_noise ^= g_noise << 5;
    return (float)(g_noise >> 8) / 8388608.0f - 1.0f;
}

/* Differentiating the raw gyro at 500 Hz multiplies sensor noise by 1/dt before
 * the low-pass ever sees it, and the result lands on four servos that then chase
 * it for the whole hold to apogee - current, heat and gear wear bought with
 * nothing, since what damps this airframe is its own aerodynamics. The loop has
 * to stay quiet under a noisy measurement. */
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
            /* Held above settle_rate_rads so this stays in DAMP and measures
             * the inner loop alone. */
            .roll_rate_rads = 0.5f + 0.01f * noise_step(),
            .maneuver_enabled = true,
        };
        float d = roll_ctrl_step(&rc, &in);
        if (i > 0) travel += fabsf(d - prev);
        prev = d;
    }
    assert(travel / (float)(n - 1) < 1e-3f);
}

/* The slew limit is what keeps a step in the command from being handed to four
 * servos as a step. It only binds on large, fast changes, which the tuned loop
 * no longer produces at ordinary excitation - so it needs its own excitation
 * rather than riding along in the regression pin. */
static void test_slew_limit_bounds_command_rate(void)
{
    roll_ctrl_config_t cfg = test_cfg();
    roll_ctrl_t rc;
    roll_ctrl_init(&rc, &cfg);

    const float max_step = cfg.deflection_slew_rads * cfg.dt_s;
    float prev = 0.0f;
    bool  ever_bound = false;

    for (int i = 0; i < 200; i++) {
        /* Sign flips every 20 ticks: a demand the loop chases hard in one
         * direction, then hard the other way. */
        float rate = ((i / 20) % 2 == 0) ? 8.0f : -8.0f;
        roll_ctrl_input_t in = { .q = quat_identity(), .q_valid = true,
                                 .roll_rate_rads = rate, .maneuver_enabled = true };
        float d = roll_ctrl_step(&rc, &in);
        assert(fabsf(d - prev) <= max_step + 1e-6f);
        if (fabsf(fabsf(d - prev) - max_step) < 1e-7f) ever_bound = true;
        prev = d;
    }
    assert(ever_bound);   /* otherwise the bound above proves nothing */
}

/* Rejecting a constant roll moment in HOLD needs no integrator on the angle.
 * The rate integrator drives the rate error to zero, and with the vehicle at
 * rest the only fixed point of that is rate_sp = 0, which the outer loop only
 * produces at zero angle error. So the trim is cancelled at the angle even
 * though nothing integrates the angle. */
static void test_hold_rejects_constant_trim(void)
{
    roll_ctrl_config_t cfg = test_cfg();
    roll_ctrl_t rc;
    roll_ctrl_init(&rc, &cfg);
    plant_t pl;
    plant_init(&pl, 0.0f);
    pl.bias = 0.3f;

    run(&rc, &pl, CTRL_HZ * 10, true, NULL);
    assert(rc.phase == ROLL_CTRL_HOLD);

    /* Within a quarter degree of the 90 deg target, with the bias still on. */
    assert(fabsf(rc.theta_meas - cfg.target_angle_rad) < 0.0044f);
    /* And holding it, not drifting through it. */
    assert(fabsf(pl.p) < 0.02f);
}

/* Sensor faults are transient by nature and the coast is over seven seconds
 * long, so ending the maneuver on one is throwing away the objective to avoid a
 * risk that has already passed. Recovery has to resume the *same* maneuver
 * though: capturing a fresh reference would rotate another 90 deg from wherever
 * the airframe happens to point by then, which is not what was asked for. */
static void test_data_fault_abort_recovers_against_the_same_reference(void)
{
    roll_ctrl_config_t cfg = test_cfg();
    roll_ctrl_t rc;
    roll_ctrl_init(&rc, &cfg);
    plant_t pl;
    plant_init(&pl, 0.0f);

    for (int i = 0; i < CTRL_HZ * 6 && rc.phase != ROLL_CTRL_ROTATE; i++)
        run(&rc, &pl, 1, true, NULL);
    assert(rc.phase == ROLL_CTRL_ROTATE);
    quat_t ref = rc.q_ref;

    for (unsigned i = 0; i < cfg.abort_debounce_ticks; i++) {
        roll_ctrl_input_t bad = { .q = pl.q, .q_valid = false,
                                  .roll_rate_rads = pl.p, .maneuver_enabled = true };
        plant_step(&pl, roll_ctrl_step(&rc, &bad));
    }
    assert(rc.phase == ROLL_CTRL_ABORT);

    run(&rc, &pl, CTRL_HZ * 6, true, NULL);          /* estimator comes back */

    assert(rc.phase == ROLL_CTRL_HOLD);
    assert(rc.q_ref.w == ref.w && rc.q_ref.x == ref.x &&
           rc.q_ref.y == ref.y && rc.q_ref.z == ref.z);
    assert(fabsf(rc.theta_meas - cfg.target_angle_rad) < 0.09f);
}

/* The rate abort is not a sensor complaint. Past 720 deg/s something is wrong
 * with the airframe or with the sign of the servo channels, and neither heals;
 * retrying would just re-enter the same divergence with a running start. */
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
    assert(rc.phase == ROLL_CTRL_ABORT);

    plant_init(&pl, 0.0f);                 /* perfectly healthy from here on */
    run(&rc, &pl, CTRL_HZ * 6, true, NULL);
    assert(rc.phase == ROLL_CTRL_ABORT);
}

/* Damping command after n ticks at a given dynamic pressure, from a fresh
 * controller. The rate is small on purpose: the whole command is linear in the
 * scheduled gain, so the ratio between two runs is exact right up until the
 * slew limiter binds, and a large error makes the higher-gain run hit it. */
static float damp_command_at(float qbar_pa, int ticks)
{
    roll_ctrl_config_t cfg = test_cfg();
    roll_ctrl_t rc;
    roll_ctrl_init(&rc, &cfg);

    float d = 0.0f;
    for (int i = 0; i < ticks; i++) {
        roll_ctrl_input_t in = { .q = quat_identity(), .q_valid = true,
                                 .roll_rate_rads = 0.1f, .qbar_pa = qbar_pa,
                                 .maneuver_enabled = true };
        d = roll_ctrl_step(&rc, &in);
    }
    return d;
}

/* Canard authority is proportional to dynamic pressure, so holding the loop
 * gain constant means raising the controller gain as qbar falls. Without it the
 * gains describe the airframe for about one second of a seven second coast. */
static void test_gain_scales_inversely_with_dynamic_pressure(void)
{
    roll_ctrl_config_t cfg = test_cfg();

    /* Two ticks: past the first low-pass sample, before the integrator matters. */
    float at_ref     = damp_command_at(cfg.qbar_ref_pa, 2);
    float at_quarter = damp_command_at(cfg.qbar_ref_pa * 0.25f, 2);

    assert(fabsf(at_ref) > 1e-4f);
    /* Four times the gain, so four times the command, while both are far from
     * the deflection limit. */
    assert(fabsf(at_quarter / at_ref - 4.0f) < 0.05f);
}

/* The ratio diverges as qbar goes to zero. Unclamped, the last second before
 * apogee would be spent amplifying gyro noise into the stops. */
static void test_gain_scale_is_clamped(void)
{
    roll_ctrl_config_t cfg = test_cfg();
    roll_ctrl_t rc;
    roll_ctrl_init(&rc, &cfg);

    /* Above the floor, so the scheduler runs, but far enough below the
     * reference that an unclamped ratio would be enormous. */
    roll_ctrl_input_t in = { .q = quat_identity(), .q_valid = true,
                             .roll_rate_rads = 0.4f,
                             .qbar_pa = cfg.qbar_floor_pa * 1.01f,
                             .maneuver_enabled = true };
    roll_ctrl_step(&rc, &in);
    assert(rc.gain_scale <= cfg.gain_scale_max + 1e-6f);
    assert(rc.gain_scale >= cfg.gain_scale_min - 1e-6f);
}

/* Near apogee there is no authority left to chase gyro noise with, so the
 * proportional path is dropped - but the trim correction is not. A standing
 * trim and the canards that oppose it both scale with dynamic pressure, so the
 * deflection that cancels the trim stays correct all the way to apogee, and
 * releasing it would let the disturbance walk the airframe off the target for
 * the rest of the coast. The command moves under the slew limit, not in a step. */
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
        assert(fabsf(d - prev) <= cfg.deflection_slew_rads * cfg.dt_s + 1e-6f
               || i == 0);
        prev = d;
    }
    assert(rc.below_qbar_floor);
    /* Settled on the integrator, which is the only term still contributing. */
    assert(fabsf(d - rc.rate_integrator) < 1e-4f);
    /* And that term is real: the rate error above was one-sided, so DAMP wound
     * it up rather than leaving it at zero. */
    assert(fabsf(rc.rate_integrator) > 1e-3f);
}

/* A caller with no airspeed estimate passes zero, which is the absence of a
 * measurement rather than a measurement of zero. Standing the canards down on
 * that would turn a missing field into a silently disabled controller. */
static void test_absent_dynamic_pressure_disables_scheduling(void)
{
    roll_ctrl_config_t cfg = test_cfg();
    roll_ctrl_t rc;
    roll_ctrl_init(&rc, &cfg);

    roll_ctrl_input_t in = { .q = quat_identity(), .q_valid = true,
                             .roll_rate_rads = 0.4f, .qbar_pa = 0.0f,
                             .maneuver_enabled = true };
    float d = roll_ctrl_step(&rc, &in);

    assert(!rc.below_qbar_floor);
    assert(rc.gain_scale == 1.0f);
    assert(fabsf(d) > 1e-5f);
}

/* The integrator answers the same plant the proportional term does, so it has
 * to be scheduled with it. Left unscaled, the loop's integral time stretches by
 * gain_scale - up to eight seconds at the clamp, longer than the whole coast -
 * and the term that exists to cancel a standing trim stops arriving in time to
 * do it. */
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

    assert(fabsf(wound[0]) > 1e-5f);
    assert(fabsf(wound[1] / wound[0] - 4.0f) < 0.05f);
}

/* A rate excursion violent enough to abort is exactly the one that runs the
 * gyro into its full-scale limit, so the run that trips the threshold can end
 * on an invalid sample rather than an over-limit one. Deciding the latch on the
 * triggering tick alone lets the airframe that most needs the latch slip
 * through it and try again. */
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
    /* The sensor gives up at the peak: the tick that trips the threshold is a
     * data fault, not an over-limit reading. */
    roll_ctrl_input_t bad = { .q = pl.q, .q_valid = true, .roll_rate_rads = NAN,
                              .maneuver_enabled = true };
    roll_ctrl_step(&rc, &bad);
    assert(rc.phase == ROLL_CTRL_ABORT);

    plant_init(&pl, 0.0f);
    run(&rc, &pl, CTRL_HZ * 6, true, NULL);
    assert(rc.phase == ROLL_CTRL_ABORT);
}

/* An estimator that drops out every few ticks must not be able to drive the
 * sequencer back and forth across ABORT for the whole coast: each recovery
 * restarts the profile while every abort brakes to zero, so the maneuver makes
 * no progress and four servos pay for it. Recovery therefore demands a much
 * longer run of healthy data than the abort demands of bad data. */
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
    assert(aborts <= 2);
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
    test_abort_on_excess_rate();
    test_no_authority_still_reaches_hold();
    test_damp_regression_pin();
    printf("roll_ctrl: OK\n");
    return 0;
}
