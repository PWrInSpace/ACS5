#include "control/roll_ctrl.h"
#include "control/axes.h"
#include "control/control_rate.h"
#include "utils/mathx.h"   /* clampf */
#include <math.h>

void roll_profile_init(roll_profile_t *p, float target_rad,
                       float max_rate_rads, float max_accel_rads2)
{
    *p = (roll_profile_t){0};
    p->target    = target_rad;
    p->max_rate  = max_rate_rads;
    p->max_accel = max_accel_rads2;
}

void roll_profile_step(roll_profile_t *p, float dt_s)
{
    if (p->done || dt_s <= 0.0f) return;

    float remaining = p->target - p->theta;
    float dir       = (remaining >= 0.0f) ? 1.0f : -1.0f;

    /* Chase the rate that brakes to zero exactly at the target
     * (v = sqrt(2*a*d)); the accel-limited slew toward it traces the
     * trapezoid and cannot overshoot. */
    float stop_rate = sqrtf(2.0f * p->max_accel * fabsf(remaining));
    float rate_des  = dir * fminf(stop_rate, p->max_rate);
    float step      = p->max_accel * dt_s;

    p->rate   = clampf(rate_des, p->rate - step, p->rate + step);
    p->theta += p->rate * dt_s;

    /* Discrete integration can land a hair past the target; clamping keeps
     * the setpoint monotonic and stops dir from flipping. */
    if ((dir > 0.0f && p->theta > p->target) ||
        (dir < 0.0f && p->theta < p->target))
        p->theta = p->target;

    /* Once theta sits on the target the slew drives rate to zero by itself.
     * Snapping the remainder discards up to 1e-3 rad/s on top of that tick's
     * full slew step, so the terminating tick is the one place the setpoint
     * rate moves by more than max_accel*dt_s. */
    if (fabsf(p->target - p->theta) < 0.005f && fabsf(p->rate) < 1e-3f) {
        p->theta = p->target;
        p->rate  = 0.0f;
        p->done  = true;
    }
}

static const float k_roll_axis[3] = { [BODY_AXIS_ROLL] = 1.0f };

roll_ctrl_config_t roll_ctrl_config_default(void)
{
    /* Tuned against the airframe extracted from joust1_prot3.ork: 400/400 across
     * two Monte Carlo seeds at +-50% on C_ld, C_lp and I_xx, worst case 0.4 deg
     * overshoot and 2.7 s to settle against a 3 s budget.
     *
     * The gains sit at the LOW edge of the passing region, not its middle. The
     * region runs well past kp_rate 0.16 without the simulator showing any
     * degradation, which does not mean those gains are safe - it means the model
     * has no linkage backlash, no structural compliance and no measurement delay,
     * so it cannot see an upper stability bound at all. That bound has to come
     * off the bench. Everything below is provisional until it does. */
    return (roll_ctrl_config_t){
        .dt_s                    = CONTROL_DT_S,
        .target_angle_rad        = 1.5707963f,   /* +90 deg */
        .settle_rate_rads        = 0.1745f,      /* 10 deg/s */
        .settle_time_s           = 0.5f,
        .profile_max_rate_rads   = 1.8326f,      /* 105 deg/s */
        .profile_max_accel_rads2 = 3.6652f,      /* 210 deg/s^2 */
        .abort_rate_rads         = 12.566f,      /* 720 deg/s */
        .abort_debounce_ticks    = (int)SEC_TO_TICKS(0.016f),
        .recover_debounce_ticks  = (int)SEC_TO_TICKS(0.25f),
        .kp_angle                = 2.5f,
        .rate_limit_rads         = 2.0944f,      /* 120 deg/s */
        .kp_rate                 = 0.10f,
        /* Integral time kp_rate/ki_rate is 1 s: slow next to the rate loop, fast
         * next to the coast. Retuning kp_rate must carry this with it. */
        .ki_rate                 = 0.10f,
        /* Has to exceed the worst credible trim: half a degree of aft-fin
         * misalignment needs about 4 deg of canard to cancel on this airframe. */
        .rate_int_limit_rad      = 0.1745f,      /* 10 deg */
        /* Burnout conditions from the OpenRocket model: 77.6 m/s at 148 m. */
        .qbar_ref_pa             = 3634.0f,
        .gain_scale_min          = 0.5f,
        .gain_scale_max          = 8.0f,
        /* About 16 m/s, roughly 1.3 s short of apogee. Past it the canards have
         * a twenty-fourth of their burnout authority and the clamp has already
         * run out of room to compensate. */
        .qbar_floor_pa           = 150.0f,
        .target_lpf_alpha        = 0.30f,
        .max_deflection_rad      = 0.2618f,      /* 15 deg */
        /* 400 deg/s = 4.44 us/ms of horn. The servo_t75 driver limits to an
         * effective 4.00 us/ms downstream, so in firmware this never binds -
         * it only shapes the command in host tests, which have no driver. */
        .deflection_slew_rads    = 6.9813f,
    };
}

/* Clears every state field and leaves cfg untouched. roll_ctrl_step runs this on
 * each disarmed tick. Every state field rests at zero (ROLL_CTRL_IDLE included)
 * except q_ref, so rebuilding the struct keeps new fields covered for free. */
static void roll_ctrl_reset_state(roll_ctrl_t *rc)
{
    const roll_ctrl_config_t cfg = rc->cfg;
    *rc = (roll_ctrl_t){ .cfg = cfg, .q_ref = quat_identity() };
}

void roll_ctrl_init(roll_ctrl_t *rc, const roll_ctrl_config_t *cfg)
{
    rc->cfg = *cfg;
    /* A threshold of 0 makes the debounce comparison false before the counter
     * can ever move, which disables every abort path - the opposite of what
     * "no debounce" is meant to say. 1 is that meaning. */
    if (rc->cfg.abort_debounce_ticks == 0)
        rc->cfg.abort_debounce_ticks = 1;
    roll_ctrl_reset_state(rc);
}

/* Inner rate loop: PI on the roll rate, output low-passed then slew-limited,
 * driving all four canards together.
 *
 * No derivative term. Differentiating the raw gyro at this rate multiplies
 * sensor noise by 1/dt_s before the low-pass sees any of it, and the airframe
 * supplies its own damping through C_lp - the phase lead bought nothing the
 * simulator could measure and cost continuous servo travel through the hold.
 * If bench testing shows the linkage is slower than the datasheet implies,
 * that is the point to reconsider, on a filtered rate rather than a raw one.
 *
 * The gains are unvalidated - there is no flight data; a simulator sweep and a
 * bench run are what qualify them. */
static float inner_rate_loop(roll_ctrl_t *rc, float rate_sp, float p_meas)
{
    const roll_ctrl_config_t *c = &rc->cfg;

    rc->p_prev = p_meas;   /* the bad-gyro path substitutes this next tick */

    /* Both proportional and integral terms scale: they answer the same plant,
     * and leaving the integrator behind would change the loop's integral time
     * with every change in airspeed. The integrator clamp does not scale - it
     * is a deflection limit, and deflection does not care about airspeed. */
    float error = rate_sp - p_meas;
    float raw   = c->kp_rate * rc->gain_scale * error;

    rc->filtered_target = c->target_lpf_alpha * raw
                        + (1.0f - c->target_lpf_alpha) * rc->filtered_target;

    /* The integrator sits outside the low-pass: it is already the slow path, and
     * filtering it only adds lag to the term that cancels the trim.
     *
     * Conditional integration, because the term exists to cancel a standing trim
     * and nothing else. Entering free flight spinning, or recovering from a
     * dropped gyro sample, presents an error tens of times the steady-state one;
     * integrating that drives the accumulator into its clamp, and unwinding it
     * afterwards throws the rate through zero and rings. Accumulation therefore
     * stops whenever the command already sits against the deflection limit and
     * the error would push it further in - at that point more integral buys no
     * additional authority, only recovery time. */
    float unsat = rc->filtered_target + rc->rate_integrator;
    bool  driving_deeper = (unsat >=  c->max_deflection_rad && error > 0.0f) ||
                           (unsat <= -c->max_deflection_rad && error < 0.0f);
    /* Frozen through the rotation: there the error is dominated by lag behind
     * the moving profile, which is not a disturbance and reverses sign when the
     * profile decelerates. What DAMP learned about the trim stays valid across
     * the two seconds of the maneuver. */
    if (!driving_deeper && rc->phase != ROLL_CTRL_ROTATE)
        rc->rate_integrator = clampf(rc->rate_integrator
                                         + c->ki_rate * rc->gain_scale * error * c->dt_s,
                                     -c->rate_int_limit_rad, c->rate_int_limit_rad);

    float target = clampf(rc->filtered_target + rc->rate_integrator,
                          -c->max_deflection_rad, c->max_deflection_rad);
    float max_step = c->deflection_slew_rads * c->dt_s;
    rc->current_delta += clampf(target - rc->current_delta, -max_step, max_step);
    return rc->current_delta;
}

/* Accumulate the maneuver angle across wraps so the outer loop sees a
 * continuous error even though the twist is only known modulo 2*pi.
 *
 * Callers must skip this entirely on a quaternion they have rejected: NaN is
 * absorbing here (it reaches theta_meas and theta_prev_wrapped, and within an
 * armed window only the DAMP->ROTATE edge reassigns either), and clampf does
 * not stop it because both of its comparisons are false on NaN.
 * Skipping freezes theta_prev_wrapped for the duration of the fault. Before the
 * debounce expires that gap is at most abort_debounce_ticks-1 ticks (14 ms at
 * the default 8); afterwards the sequencer is in ABORT and the gap runs until
 * recovery, which is why the resume path unwraps explicitly before re-aiming
 * the profile. Either way the unwrap mistracks only if more than pi accumulates
 * across the gap, and ABORT spends that whole window damping the rate toward
 * zero. Below the pi threshold theta_meas catches up on the first good tick. */
static void update_theta(roll_ctrl_t *rc, quat_t q)
{
    quat_t q_err   = quat_mul(quat_conj(rc->q_ref), q);
    float  wrapped = quat_twist_angle(q_err, k_roll_axis);
    rc->theta_meas += quat_wrap_pi(wrapped - rc->theta_prev_wrapped);
    rc->theta_prev_wrapped = wrapped;
}

float roll_ctrl_step(roll_ctrl_t *rc, const roll_ctrl_input_t *in)
{
    const roll_ctrl_config_t *c = &rc->cfg;

    if (!in->maneuver_enabled) {
        /* Disarmed (pre-arm or post-apogee): neutral canards, fresh state. */
        roll_ctrl_reset_state(rc);
        return 0.0f;
    }

    /* Gyro and attitude validity are separate concerns: a bad quaternion
     * ends the maneuver (there is no angle control without attitude), but
     * it must not blind the damper, which only ever needed the gyro. */
    bool gyro_ok = isfinite(in->roll_rate_rads);
    bool att_ok  = in->q_valid && quat_finite(in->q);

    /* Scheduling on dynamic pressure. A non-positive qbar is the absence of a
     * measurement, not a measurement of zero: scheduling off, gain unchanged,
     * floor not applied, so a caller that cannot supply airspeed degrades to
     * the fixed-gain controller instead of to no controller at all. */
    bool qbar_known = isfinite(in->qbar_pa) && in->qbar_pa > 0.0f;
    rc->gain_scale = qbar_known
                   ? clampf(c->qbar_ref_pa / in->qbar_pa,
                            c->gain_scale_min, c->gain_scale_max)
                   : 1.0f;
    rc->below_qbar_floor = qbar_known && in->qbar_pa < c->qbar_floor_pa;

    /* NaN compares false, so this needs a healthy gyro to mean anything. */
    bool overspeed = gyro_ok && fabsf(in->roll_rate_rads) > c->abort_rate_rads;

    /* Sensor faults are overwhelmingly transient - one NaN, one dropped IMU
     * frame, a momentary estimator re-convergence - and at 500 Hz a
     * single-sample latch would turn a 2 ms blip into permanent loss of the
     * maneuver. Requiring N consecutive faulted ticks still aborts within
     * N*2 ms of a genuine persistent fault, negligible against the 1.36 s
     * maneuver.
     *
     * Nor is the abort itself final, for data faults. The coast is over seven
     * seconds and the maneuver needs about two, so a blip that has already
     * passed is no reason to spend the rest of the flight damping. An
     * over-limit rate is different in kind: past that threshold the airframe or
     * the sign of the servo channels is wrong, neither of which heals, and a
     * retry would re-enter the same divergence with a running start. */
    if (!gyro_ok || !att_ok || overspeed) {
        rc->recover_count = 0;
        /* Sticky across the whole fault run, not just its last tick. The
         * excursion that trips this threshold is the one that drives the gyro
         * into its full-scale limit, so the run can end on an invalid sample
         * rather than an over-limit one - and deciding on that tick alone would
         * let precisely the airframe this latch exists for slip through and
         * try again. */
        if (overspeed) rc->overspeed_seen = true;
        /* Saturate rather than keep counting: there is nothing beyond the
         * threshold left to count and nothing to wrap. */
        if (rc->abort_count < c->abort_debounce_ticks &&
            ++rc->abort_count >= c->abort_debounce_ticks) {
            if (rc->phase != ROLL_CTRL_ABORT)
                rc->resume_phase = (rc->phase == ROLL_CTRL_ROTATE ||
                                    rc->phase == ROLL_CTRL_HOLD)
                                 ? ROLL_CTRL_ROTATE : ROLL_CTRL_DAMP;
            rc->phase = ROLL_CTRL_ABORT;
            if (rc->overspeed_seen) rc->abort_latched = true;
        }
    } else {
        rc->abort_count    = 0;
        rc->overspeed_seen = false;
        /* Recovery is deliberately far slower than the abort. With one shared
         * threshold an estimator dropping out every few ticks walks the
         * sequencer across ABORT and back indefinitely: each recovery restarts
         * the profile from a standstill while every abort brakes to zero, so
         * the maneuver makes no progress and the servos pay for all of it. */
        if (rc->phase == ROLL_CTRL_ABORT && !rc->abort_latched &&
            ++rc->recover_count >= c->recover_debounce_ticks) {
            rc->recover_count = 0;
            rc->phase = rc->resume_phase;
            if (rc->phase == ROLL_CTRL_ROTATE) {
                /* Same q_ref, same target - the maneuver is resumed, not
                 * restarted. update_theta first, so whatever the vehicle
                 * rotated while the estimate was rejected is accounted for
                 * rather than dropped: theta_meas is what post-flight
                 * identification reads, and a silently short angle there is
                 * worse than a wrong one that is visible. The unwrap is exact
                 * up to pi of accumulated rotation, and ABORT spends that whole
                 * window damping the rate toward zero.
                 *
                 * The profile is then re-aimed from the angle actually reached,
                 * so it ramps out the remainder instead of replaying the whole
                 * 90 deg. */
                update_theta(rc, in->q);
                roll_profile_init(&rc->profile, c->target_angle_rad,
                                  c->profile_max_rate_rads, c->profile_max_accel_rads2);
                rc->profile.theta = rc->theta_meas;
            } else {
                rc->settle_timer_s = 0.0f;
            }
        }
    }

    /* Hold the last good sample on a bad gyro. Substituting zero would tell the
     * loop the airframe had stopped rotating, and it would unwind the damping
     * command it is holding for exactly as long as the fault lasts. The last
     * measurement is the best available estimate of a rate that cannot change
     * much over a debounce window. p_prev is still 0 if the very first tick
     * faults. */
    float p = gyro_ok ? in->roll_rate_rads : rc->p_prev;

    switch (rc->phase)
    {
        case ROLL_CTRL_IDLE:
            rc->phase = ROLL_CTRL_DAMP;
            rc->settle_timer_s = 0.0f;
            /* fall through */
        case ROLL_CTRL_DAMP:
            rc->theta_sp = 0.0f;
            if (fabsf(p) < c->settle_rate_rads) {
                rc->settle_timer_s += c->dt_s;
                /* att_ok gates the transition, not the timer: with the gyro
                 * healthy the settle gate passes on real data, so a rejected
                 * quaternion arriving on the last tick of the window would
                 * otherwise be latched into q_ref - and every later
                 * update_theta drags it into theta_meas through
                 * quat_conj(q_ref), past the point an input check can help. */
                if (rc->settle_timer_s >= c->settle_time_s && att_ok) {
                    rc->phase              = ROLL_CTRL_ROTATE;
                    rc->q_ref              = in->q;
                    rc->theta_meas         = 0.0f;
                    rc->theta_prev_wrapped = 0.0f;
                    roll_profile_init(&rc->profile, c->target_angle_rad,
                                      c->profile_max_rate_rads,
                                      c->profile_max_accel_rads2);
                }
            } else {
                rc->settle_timer_s = 0.0f;
            }
            break;

        case ROLL_CTRL_ROTATE:
            if (att_ok) update_theta(rc, in->q);
            roll_profile_step(&rc->profile, c->dt_s);
            rc->theta_sp = rc->profile.theta;
            /* The profile is a deterministic ramp, so it always completes and
             * ROTATE always terminates - there is nothing left for a watchdog to
             * catch here. Whatever the rotation did not close, HOLD closes with
             * the angle loop and the integrator; abandoning a near-miss to plain
             * damping would throw away the maneuver to avoid finishing it
             * imperfectly. ABORT stays for faults that actually invalidate
             * angle control. */
            if (rc->profile.done)
                rc->phase = ROLL_CTRL_HOLD;
            break;

        case ROLL_CTRL_HOLD:
            if (att_ok) update_theta(rc, in->q);
            rc->theta_sp = c->target_angle_rad;
            break;

        case ROLL_CTRL_ABORT:
            break;
    }

    /* ABORT is pure damping, so the logged setpoint must read as such. Placed
     * after the switch because a phase that transitions in-switch leaves
     * theta_sp already set to whatever that phase was tracking. */
    if (rc->phase == ROLL_CTRL_ABORT)
        rc->theta_sp = 0.0f;

    rc->rate_sp = 0.0f;   /* DAMP and ABORT: pure damping */
    if (rc->phase == ROLL_CTRL_ROTATE || rc->phase == ROLL_CTRL_HOLD) {
        float ff = (rc->phase == ROLL_CTRL_ROTATE) ? rc->profile.rate : 0.0f;
        rc->rate_sp = clampf(c->kp_angle * (rc->theta_sp - rc->theta_meas) + ff,
                             -c->rate_limit_rads, c->rate_limit_rads);
    }

    /* Below the floor the loop holds its learned trim correction and nothing
     * else. A standing trim moment and the canard authority that opposes it are
     * both proportional to qbar, so the deflection that cancels the trim is the
     * same at every airspeed - going to neutral here would hand the airframe
     * back to the disturbance for the rest of the coast, which on a 2 deg trim
     * is over ten degrees of drift off the target. What is worth dropping is
     * the proportional path, whose scheduled gain would otherwise spend the
     * last seconds chasing gyro noise it has no authority to act on.
     *
     * The sequencer keeps running so the log still shows which phase ran out. */
    if (rc->below_qbar_floor) {
        /* Still the substitute value the bad-gyro path feeds back, and still
         * what the settle gate would see, so it cannot be left behind here. */
        rc->p_prev = p;
        float target = clampf(rc->rate_integrator,
                              -c->max_deflection_rad, c->max_deflection_rad);
        float max_step = c->deflection_slew_rads * c->dt_s;
        rc->current_delta += clampf(target - rc->current_delta, -max_step, max_step);
        return rc->current_delta;
    }

    return inner_rate_loop(rc, rc->rate_sp, p);
}
