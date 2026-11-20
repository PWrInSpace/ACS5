#include "control/roll_ctrl.h"
#include "config/airframe.h"
#include "control/axes.h"
#include "utils/control_rate.h"
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

    /* Chase the rate that brakes to zero exactly at the target (v = sqrt(2*a*d));
     * the accel-limited slew toward it traces the trapezoid. */
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

    /* The snap makes the terminating tick the one place the setpoint rate may move
     * by more than max_accel*dt_s (up to 1e-3 rad/s extra). */
    if (fabsf(p->target - p->theta) < 0.005f && fabsf(p->rate) < 1e-3f) {
        p->theta = p->target;
        p->rate  = 0.0f;
        p->done  = true;
    }
}

static const float k_roll_axis[3] = { [BODY_AXIS_ROLL] = 1.0f };

roll_ctrl_config_t roll_ctrl_config_default(void)
{
    /* Provisional gains, tuned in simulation only: docs/design.md#roll-gain-tuning */
    return (roll_ctrl_config_t){
        .dt_s                    = CONTROL_DT_S,
        .target_angle_rad        = 1.5707963f,   /* +90 deg */
        .settle_rate_rads        = 0.1745f,      /* 10 deg/s */
        .settle_time_s           = 0.5f,
        .profile_max_rate_rads   = AIRFRAME_ROLL_PROFILE_MAX_RATE_RADS,
        .profile_max_accel_rads2 = AIRFRAME_ROLL_PROFILE_MAX_ACCEL_RADS2,
        .abort_rate_rads         = 12.566f,      /* 720 deg/s */
        .abort_debounce_ticks    = (int)SEC_TO_TICKS(0.016f),
        .recover_debounce_ticks  = (int)SEC_TO_TICKS(0.25f),
        .kp_angle                = AIRFRAME_ROLL_KP_ANGLE,
        .rate_limit_rads         = 2.0944f,      /* 120 deg/s */
        .kp_rate                 = AIRFRAME_ROLL_KP_RATE,
        .ki_rate                 = AIRFRAME_ROLL_KI_RATE,
        .rate_int_limit_rad      = AIRFRAME_ROLL_RATE_INT_LIMIT_RAD,
        .qbar_ref_pa             = AIRFRAME_QBAR_REF_PA,
        .gain_scale_min          = 0.5f,
        .gain_scale_max          = 8.0f,
        .qbar_floor_pa           = AIRFRAME_QBAR_FLOOR_PA,
        .target_lpf_alpha        = 0.30f,
        .max_deflection_rad      = AIRFRAME_MAX_DEFLECTION_RAD,
        /* 400 deg/s = 4.44 us/ms of horn; servo_t75 limits to 4.00 us/ms, so this binds only in host tests. */
        .deflection_slew_rads    = 6.9813f,
    };
}

/* Clears every state field and keeps cfg; all state rests at zero except q_ref. */
static void roll_ctrl_reset_state(roll_ctrl_t *rc)
{
    const roll_ctrl_config_t cfg = rc->cfg;
    *rc = (roll_ctrl_t){ .cfg = cfg, .q_ref = quat_identity() };
}

void roll_ctrl_init(roll_ctrl_t *rc, const roll_ctrl_config_t *cfg)
{
    rc->cfg = *cfg;
    /* 0 would make the debounce compare false before the counter moves, disabling
     * every abort path; 1 is what "no debounce" means. */
    if (rc->cfg.abort_debounce_ticks == 0)
        rc->cfg.abort_debounce_ticks = 1;
    roll_ctrl_reset_state(rc);
}

/* Inner rate loop: PI on the roll rate, output low-passed then slew-limited,
 * driving all four canards together. Why no D term: docs/design.md#no-derivative-term */
static float inner_rate_loop(roll_ctrl_t *rc, float rate_sp, float p_meas)
{
    const roll_ctrl_config_t *c = &rc->cfg;

    rc->p_prev = p_meas;   /* the bad-gyro path substitutes this next tick */

    /* P and I both scale so the integral time stays fixed across airspeed; the
     * integrator clamp is a deflection limit and does not scale. */
    float error = rate_sp - p_meas;
    float raw   = c->kp_rate * rc->gain_scale * error;

    rc->filtered_target = c->target_lpf_alpha * raw
                        + (1.0f - c->target_lpf_alpha) * rc->filtered_target;

    /* Integrator outside the low-pass, and conditional: docs/design.md#rate-integrator */
    float unsat = rc->filtered_target + rc->rate_integrator;
    bool  driving_deeper = (unsat >=  c->max_deflection_rad && error > 0.0f) ||
                           (unsat <= -c->max_deflection_rad && error < 0.0f);
    /* Frozen in ROTATE: the error there is lag behind the moving profile, not trim. */
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

/* Accumulate the maneuver angle across wraps so the outer loop sees a continuous error.
 * Callers must skip it on a rejected quaternion: docs/design.md#angle-unwrap-across-faults */
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

    /* A bad quaternion ends angle control but must not blind the damper, which needs only the gyro. */
    bool gyro_ok = isfinite(in->roll_rate_rads) && !in->rate_stale;
    bool att_ok  = in->q_valid && quat_finite(in->q);

    /* qbar <= 0 means no measurement: fixed gains and no floor, not a stood-down controller. */
    bool qbar_known = isfinite(in->qbar_pa) && in->qbar_pa > 0.0f;
    rc->gain_scale = qbar_known
                   ? clampf(c->qbar_ref_pa / in->qbar_pa,
                            c->gain_scale_min, c->gain_scale_max)
                   : 1.0f;
    rc->below_qbar_floor = qbar_known && in->qbar_pa < c->qbar_floor_pa;

    /* NaN compares false, so this needs a healthy gyro to mean anything. */
    bool overspeed = gyro_ok && fabsf(in->roll_rate_rads) > c->abort_rate_rads;

    /* Debounced abort; data faults recover, overspeed latches: docs/design.md#fault-debounce-and-recovery */
    if (!gyro_ok || !att_ok || overspeed) {
        rc->recover_count = 0;
        /* Sticky across the fault run: an overspeed drives the gyro to full scale, so
         * the run can end on an invalid sample rather than an over-limit one. */
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
        /* Recovery is far slower than abort, so an intermittent fault cannot thrash ABORT. */
        if (rc->phase == ROLL_CTRL_ABORT && !rc->abort_latched &&
            ++rc->recover_count >= c->recover_debounce_ticks) {
            rc->recover_count = 0;
            rc->phase = rc->resume_phase;
            if (rc->phase == ROLL_CTRL_ROTATE) {
                /* Resumed, not restarted: count the rotation made during the fault, then
                 * re-aim the profile from the angle actually reached. */
                update_theta(rc, in->q);
                roll_profile_init(&rc->profile, c->target_angle_rad,
                                  c->profile_max_rate_rads, c->profile_max_accel_rads2);
                rc->profile.theta = rc->theta_meas;
            } else {
                rc->settle_timer_s = 0.0f;
            }
        }
    }

    /* Hold the last good rate on a bad gyro; substituting zero would unwind the damping
     * command for the fault's duration. p_prev is 0 if the very first tick faults. */
    float p = gyro_ok ? in->roll_rate_rads : rc->p_prev;

    /* Damping on a frozen rate would drive the integrator to its clamp and hold
     * the canards there open-loop; with nothing to damp on, go neutral. */
    if (rc->phase == ROLL_CTRL_ABORT && !gyro_ok) {
        rc->theta_sp = 0.0f;
        rc->rate_sp  = 0.0f;
        float max_step = c->deflection_slew_rads * c->dt_s;
        rc->current_delta += clampf(0.0f - rc->current_delta, -max_step, max_step);
        return rc->current_delta;
    }

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
                /* att_ok gates the transition, not the timer: a rejected quaternion
                 * latched into q_ref would corrupt every later theta_meas. */
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
            /* The profile always completes; HOLD closes whatever error ROTATE leaves. */
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

    /* ABORT logs a zero setpoint; after the switch, since an in-switch transition
     * leaves theta_sp at the previous phase's value. */
    if (rc->phase == ROLL_CTRL_ABORT)
        rc->theta_sp = 0.0f;

    rc->rate_sp = 0.0f;   /* DAMP and ABORT: pure damping */
    if (rc->phase == ROLL_CTRL_ROTATE || rc->phase == ROLL_CTRL_HOLD) {
        float ff = (rc->phase == ROLL_CTRL_ROTATE) ? rc->profile.rate : 0.0f;
        rc->rate_sp = clampf(c->kp_angle * (rc->theta_sp - rc->theta_meas) + ff,
                             -c->rate_limit_rads, c->rate_limit_rads);
    }

    /* Below the floor only the learned trim is held: docs/design.md#qbar-floor
     * The sequencer keeps running so the log still shows which phase ran out. */
    if (rc->below_qbar_floor) {
        /* p_prev still feeds the bad-gyro path and the settle gate. */
        rc->p_prev = p;
        float target = clampf(rc->rate_integrator,
                              -c->max_deflection_rad, c->max_deflection_rad);
        float max_step = c->deflection_slew_rads * c->dt_s;
        rc->current_delta += clampf(target - rc->current_delta, -max_step, max_step);
        return rc->current_delta;
    }

    return inner_rate_loop(rc, rc->rate_sp, p);
}
