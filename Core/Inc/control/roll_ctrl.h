#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "control/quat.h"

/* Trapezoidal setpoint profile.
 * Shapes the commanded roll angle so the cascade tracks a feasible
 * trajectory instead of a 90 deg step (which would saturate the canards). */
typedef struct
{
    float target;      /* rad */
    float max_rate;    /* rad/s */
    float max_accel;   /* rad/s^2 */
    float theta;       /* current setpoint, rad */
    float rate;        /* current setpoint rate, rad/s (feedforward) */
    bool  done;
} roll_profile_t;

void roll_profile_init(roll_profile_t *p, float target_rad,
                       float max_rate_rads, float max_accel_rads2);
void roll_profile_step(roll_profile_t *p, float dt_s);

/* Maneuver sequencer + cascade controller.
 * DAMP:   inner rate loop, setpoint 0 - pure rate damping.
 * ROTATE: capture q_ref, track the trapezoid profile via
 *         angle-P + rate feedforward -> inner rate PI.
 * HOLD:   angle-P at the 90 deg target, held until apogee.
 * ABORT:  pure rate damping. A data fault resumes the same maneuver after
 *         recover_debounce_ticks of healthy data; an over-limit rate latches
 *         for the rest of the armed window. */
typedef enum
{
    ROLL_CTRL_IDLE   = 0,
    ROLL_CTRL_DAMP   = 1,
    ROLL_CTRL_ROTATE = 2,
    ROLL_CTRL_HOLD   = 3,
    ROLL_CTRL_ABORT  = 4,
} roll_ctrl_phase_t;

typedef struct
{
    float dt_s;                     /* tick period; must be finite and > 0, unchecked */

    /* sequencing */
    float target_angle_rad;         /* +90 deg maneuver */
    float settle_rate_rads;         /* DAMP->ROTATE gate ... */
    float settle_time_s;            /* ... held this long */
    float profile_max_rate_rads;
    float profile_max_accel_rads2;
    float abort_rate_rads;          /* |p| above this -> ABORT (safety) */
    /* Consecutive healthy ticks before a data-fault ABORT resumes. Much longer
     * than the abort debounce on purpose: without that asymmetry an intermittent
     * fault thrashes the sequencer across ABORT for the whole coast. */
    uint16_t recover_debounce_ticks;
    uint16_t abort_debounce_ticks;  /* consecutive faulted ticks before ABORT;
                                       1 aborts on the first faulted tick.
                                       roll_ctrl_init clamps 0 up to 1 - there
                                       is no value that disables the aborts. */

    /* cascade */
    float kp_angle;                 /* outer P, [1/s] */
    float rate_limit_rads;          /* outer output clamp */
    float kp_rate;                  /* inner P */
    /* Rate-loop integrator. A steady aero trim torque otherwise leaves a
     * standing rate error, and the DAMP->ROTATE gate turns that error into a
     * pass/fail on the whole maneuver: the trim only has to exceed
     * kp_rate * settle_rate_rads of equivalent deflection to hold the sequencer
     * in DAMP for the entire coast. On this airframe a quarter degree of aft-fin
     * misalignment is already past that. */
    float ki_rate;                  /* [rad per (rad/s * s)] */
    float rate_int_limit_rad;       /* integrator clamp, deflection units */
    /* Gain scheduling on dynamic pressure. Canard authority is proportional to
     * qbar, which falls by more than an order of magnitude across this coast -
     * 3634 Pa at burnout, 350 Pa five seconds later - so a fixed gain is correct
     * at one instant of the flight and wrong at every other. Scaling the rate
     * loop by qbar_ref/qbar holds the loop gain where it was tuned.
     *
     * The clamp is not optional: the ratio diverges as qbar goes to zero, and an
     * unbounded gain near apogee would chase gyro noise into the stops. */
    float qbar_ref_pa;              /* qbar the gains were tuned at */
    float gain_scale_min, gain_scale_max;
    /* Below this the proportional path is dropped and the canards hold only the
     * integrator's trim correction: there is no longer authority worth chasing
     * noise for, but a standing trim scales with qbar exactly as the canards do,
     * so the deflection that cancels it stays valid all the way to apogee. */
    float qbar_floor_pa;
    float target_lpf_alpha;         /* inner low-pass */
    float max_deflection_rad;       /* canard hard limit */
    float deflection_slew_rads;     /* canard slew limit */
} roll_ctrl_config_t;

roll_ctrl_config_t roll_ctrl_config_default(void);

typedef struct
{
    quat_t q;                /* current attitude (body->world) */
    bool   q_valid;          /* estimator gravity-init complete + finite */
    float  roll_rate_rads;   /* body-Z gyro, rad/s */
    /* Dynamic pressure, Pa. Zero or negative means the caller has none to
     * offer: scheduling off, scale 1, floor not applied. A zeroed input struct
     * must not silently stand the canards down. */
    float  qbar_pa;
    bool   maneuver_enabled; /* From ctrl_arm: burnout plus the canard-lock delay.
                                Must be one-way per flight, and nothing here
                                enforces it: false clears the controller state, so
                                re-arming drops a latched ABORT and captures a
                                fresh q_ref - another 90 deg from wherever it then
                                points. */
} roll_ctrl_input_t;

typedef struct
{
    roll_ctrl_config_t cfg;
    roll_ctrl_phase_t  phase;

    quat_t q_ref;                 /* attitude at ROTATE entry */
    float  theta_meas;            /* unwrapped maneuver angle, rad */
    float  theta_prev_wrapped;
    float  theta_sp;              /* logged setpoint */
    float  rate_sp;               /* outer-loop output handed to the inner loop,
                                     post-clamp, rad/s */
    roll_profile_t profile;
    float  settle_timer_s;
    uint16_t abort_count;         /* consecutive faulted ticks, saturating at
                                     abort_debounce_ticks */
    uint16_t recover_count;       /* consecutive healthy ticks while in ABORT */
    bool     abort_latched;       /* rate abort: no recovery for the rest of the
                                     armed window */
    bool     overspeed_seen;      /* an over-limit rate anywhere in the current
                                     run of faulted ticks */
    roll_ctrl_phase_t resume_phase; /* where a recovered ABORT goes back to */

    /* inner-loop state */
    float  p_prev;
    float  filtered_target;
    float  current_delta;
    float  rate_integrator;
    /* Applied rate-loop multiplier. Not logged directly - the log carries baro
     * pressure, temperature and velocity, so qbar and hence this follow from it. */
    float  gain_scale;
    bool   below_qbar_floor;      /* proportional path dropped, trim held */
} roll_ctrl_t;

void  roll_ctrl_init(roll_ctrl_t *rc, const roll_ctrl_config_t *cfg);

/* One control tick of cfg.dt_s. Returns the commanded canard deflection [rad], identical
 * for all four fins. */
float roll_ctrl_step(roll_ctrl_t *rc, const roll_ctrl_input_t *in);
