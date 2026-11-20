#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "utils/quat.h"
#include "filter/mahony.h"

/* Public attitude / nav estimator facade. Call sites pull from filter/;
 * the attitude backend is currently Mahony (see filter/mahony.h). */

typedef struct
{
    float dt_s;             /* fixed loop period, s */
    float baro_tau_s;       /* complementary blend time constant on pos Z */
    float imu_max_stale_s;  /* q_valid drops after this long without IMU data */
    int   n_avg;            /* accel ring length (capped by ATTITUDE_N_AVG) */
    int   n_calib;          /* pad gravity samples before seeding quat */
    bool  require_calib;    /* flight: true; host tests may set false */
    float gyro_bias_tau_s;  /* pad gyro-bias averaging time constant */
    /* Pad stillness, required before the gyro bias learns: */
    float still_rate_max_rads; /* |gyro| per axis below this */
    float still_tilt_rad;      /* gravity direction within this of its anchor */
    float still_mag_tol;       /* |accel| within this fraction of g */
    float still_time_s;        /* ...continuously for this long */
} attitude_config_t;

attitude_config_t attitude_config_default(void);

typedef struct
{
    float accel_mps2[3];
    float gyro_rads[3];
    float mag_ut[3];
    float baro_alt_m;
    float baro_vel_z;       /* smoothed baro vertical velocity (soft-aid) */
    bool  baro_valid;       /* false = no baro sample, or one too old to fuse */
    bool  baro_vel_valid;
    bool  imu_valid;        /* false = accel/gyro are a held stale sample */
    bool  mag_valid;
    /* Accelerometer not measuring gravity (boost, coast): no accel/mag correction or gravity
     * seeding. Phrased as an inhibit so a zeroed input struct keeps standby behaviour. */
    bool  corrections_inhibited;
    bool  collecting_calib; /* STANDBY and calib not finished */
    bool  zupt_active;      /* STANDBY - clamp XY vel/pos, estimate accel bias */
} attitude_input_t;

#define ATTITUDE_N_AVG 20

typedef struct
{
    double sum[3];
    int    count;
    bool   done;
    float  mean[3];
    float  mag_mean;
} attitude_grav_calib_t;

typedef struct
{
    attitude_config_t cfg;

    /* outputs */
    quat_t q;
    bool   q_valid;
    float  acc_original[3];   /* ring-buffer-averaged body accel */
    float  acc[3];            /* world frame, gravity removed */
    float  vel[3];
    float  pos[3];
    float  ang_vel[3];        /* body rates */
    bool   rate_fresh;        /* ang_vel was measured this tick, not held */

    attitude_grav_calib_t calib;

    /* internals */
    quat_t last_good_q;
    float  prev_gyro[3];
    bool   baro_inited;
    float  baro_alpha;
    float  buf[3][ATTITUDE_N_AVG];
    int    buf_idx, buf_count;
    int    imu_stale_ticks;
    float  last_good_acc_original[3];
    float  last_good_baro_alt;
    bool   last_good_baro_alt_valid;

    mahony_t mahony;          /* attitude backend state (Mahony today) */
    float  acc_bias_w[3];
    /* Learned on the pad while still, frozen after: in flight the vertical is gyro-only. */
    float  gyro_bias[3];
    uint32_t gyro_bias_n;     /* samples averaged so far, saturating */
    float  still_anchor[3];   /* unit gravity direction where the still run began */
    bool   still_anchor_valid;
    uint32_t still_ticks;     /* consecutive still ticks, saturating */
    uint32_t still_need_ticks;
} attitude_t;

void attitude_init(attitude_t *est, const attitude_config_t *cfg);
void attitude_update(attitude_t *est, const attitude_input_t *in);

/* Soft standby reset: clears kinematics, accel bias and ring; keeps quat, calib and gyro bias. */
void attitude_reset(attitude_t *est);

/* ZYX Euler angles [rad] of the current attitude, for logging; out[2] is roll about body Z. */
void attitude_euler(const attitude_t *est, float out[3]);
