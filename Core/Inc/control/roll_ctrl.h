#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "utils/quat.h"

/* Trapezoidal roll-angle setpoint profile; a bare step would saturate the canards. */
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

/* Maneuver sequencer phases of the cascade controller. */
typedef enum
{
    ROLL_CTRL_IDLE   = 0,
    ROLL_CTRL_DAMP   = 1,  /* inner rate loop, setpoint 0 */
    ROLL_CTRL_ROTATE = 2,  /* capture q_ref, track the profile: angle-P + rate feedforward */
    ROLL_CTRL_HOLD   = 3,  /* angle-P at the target until apogee */
    ROLL_CTRL_ABORT  = 4,  /* rate damping; data faults resume, overspeed latches */
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
    /* Consecutive healthy ticks before a data-fault ABORT resumes; much longer than
     * the abort debounce: docs/design.md#fault-debounce-and-recovery */
    uint16_t recover_debounce_ticks;
    uint16_t abort_debounce_ticks;  /* consecutive faulted ticks before ABORT;
                                       roll_ctrl_init clamps 0 up to 1 */

    /* cascade */
    float kp_angle;                 /* outer P, [1/s] */
    float rate_limit_rads;          /* outer output clamp */
    float kp_rate;                  /* inner P */
    /* Rate-loop integrator; cancels standing trim: docs/design.md#rate-integrator */
    float ki_rate;                  /* [rad per (rad/s * s)] */
    float rate_int_limit_rad;       /* integrator clamp, deflection units */
    /* Rate-loop gain is scaled by qbar_ref/qbar, clamped: docs/design.md#gain-scheduling-on-dynamic-pressure */
    float qbar_ref_pa;              /* qbar the gains were tuned at */
    float gain_scale_min, gain_scale_max;
    float qbar_floor_pa;            /* below: P path dropped, trim held (docs/design.md#qbar-floor) */
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
    /* roll_rate_rads is held, not measured this tick. Phrased as stale so a zeroed
     * input struct reads as healthy. */
    bool   rate_stale;
    /* Dynamic pressure, Pa. <= 0 means none: scheduling off, scale 1, no floor, so a
     * zeroed input struct does not stand the canards down. */
    float  qbar_pa;
    /* From ctrl_arm. Must be one-way per flight, unenforced here: false clears all
     * state, so re-arming drops a latched ABORT and captures a fresh q_ref. */
    bool   maneuver_enabled;
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
    /* Applied rate-loop multiplier; not logged, recomputable from logged baro and velocity. */
    float  gain_scale;
    bool   below_qbar_floor;      /* proportional path dropped, trim held */
} roll_ctrl_t;

void  roll_ctrl_init(roll_ctrl_t *rc, const roll_ctrl_config_t *cfg);

/* One control tick of cfg.dt_s. Returns the canard deflection [rad], identical for all four fins. */
float roll_ctrl_step(roll_ctrl_t *rc, const roll_ctrl_input_t *in);
