#include "filter/attitude.h"

#include "control/control_rate.h"
#include "utils/mathx.h"

#include <math.h>

#define Ki_acc          0.00002f
#define BARO_VEL_ALPHA  0.02f
#define ACCEL_TRUST_MIN (0.7f * GRAVITY)
#define ACCEL_TRUST_MAX (1.3f * GRAVITY)

attitude_config_t attitude_config_default(void)
{
    return (attitude_config_t){
        .dt_s            = CONTROL_DT_S,
        .baro_tau_s      = 0.5f,
        .imu_max_stale_s = 0.1f,
        .n_avg           = ATTITUDE_N_AVG,
        .n_calib         = (int)SEC_TO_TICKS(20.0f),
        .require_calib   = true,
    };
}

void attitude_init(attitude_t *est, const attitude_config_t *cfg)
{
    *est = (attitude_t){0};
    est->cfg         = *cfg;
    if (est->cfg.n_avg < 1) est->cfg.n_avg = 1;
    if (est->cfg.n_avg > ATTITUDE_N_AVG) est->cfg.n_avg = ATTITUDE_N_AVG;
    if (est->cfg.n_calib < 1) est->cfg.n_calib = 1;
    est->baro_alpha  = cfg->baro_tau_s / (cfg->baro_tau_s + cfg->dt_s);
    est->q           = quat_identity();
    est->last_good_q = quat_identity();
    est->calib.mag_mean = GRAVITY;
    mahony_init(&est->mahony);
}

static quat_t quat_from_gravity(const float body_grav[3])
{
    float mag = sqrtf(body_grav[0]*body_grav[0] +
                      body_grav[1]*body_grav[1] +
                      body_grav[2]*body_grav[2]);
    if (mag < ACCEL_TRUST_MIN || mag > ACCEL_TRUST_MAX)
        return quat_identity();

    float bx = body_grav[0]/mag, by = body_grav[1]/mag, bz = body_grav[2]/mag;
    float ax = by, ay = -bx;
    float axis_mag = sqrtf(ax*ax + ay*ay);
    float angle    = acosf(clampf(bz, -1.0f, 1.0f));

    if (axis_mag < 1e-6f)
        return (bz > 0.0f) ? quat_identity() : (quat_t){0,1,0,0};

    ax /= axis_mag; ay /= axis_mag;
    float s = sinf(angle * 0.5f), c = cosf(angle * 0.5f);
    return quat_normalize((quat_t){c, ax*s, ay*s, 0.0f});
}

static bool calib_push(attitude_t *est, const float a[3])
{
    if (est->calib.done) return true;
    est->calib.sum[0] += a[0];
    est->calib.sum[1] += a[1];
    est->calib.sum[2] += a[2];
    if (++est->calib.count >= est->cfg.n_calib) {
        float inv = 1.0f / (float)est->cfg.n_calib;
        for (int i = 0; i < 3; i++)
            est->calib.mean[i] = (float)est->calib.sum[i] * inv;
        est->calib.mag_mean = sqrtf(est->calib.mean[0]*est->calib.mean[0] +
                                    est->calib.mean[1]*est->calib.mean[1] +
                                    est->calib.mean[2]*est->calib.mean[2]);
        if (est->calib.mag_mean < 0.1f) est->calib.mag_mean = GRAVITY;
        est->calib.done = true;
    }
    return est->calib.done;
}

void attitude_reset(attitude_t *est)
{
    est->vel[0] = est->vel[1] = est->vel[2] = 0.0f;
    est->pos[0] = 0.0f;
    est->pos[1] = 0.0f;
    if (est->last_good_baro_alt_valid)
        est->pos[2] = est->last_good_baro_alt;

    for (int i = 0; i < 3; i++) {
        est->last_good_vel[i] = 0.0f;
        est->last_good_pos[i] = est->pos[i];
    }

    for (int i = 0; i < 3; i++)
        for (int j = 0; j < ATTITUDE_N_AVG; j++)
            est->buf[i][j] = 0.0f;
    est->buf_idx         = 0;
    est->buf_count       = 0;
    est->imu_stale_ticks = 0;
    for (int i = 0; i < 3; i++)
        est->last_good_acc_original[i] = 0.0f;

    est->prev_gyro[0] = est->prev_gyro[1] = est->prev_gyro[2] = 0.0f;
    mahony_reset_integral(&est->mahony);
    est->acc_bias_w[0] = est->acc_bias_w[1] = est->acc_bias_w[2] = 0.0f;
    /* Preserve quaternion, calib, and mag reference (soft standby reset). */
}

void attitude_update(attitude_t *est, const attitude_input_t *in)
{
    const float dt = est->cfg.dt_s;
    const int n_avg = est->cfg.n_avg;

    if (!isfinite(dt) || dt <= 0.0001f) return;

    if (!in->imu_valid) {
        est->imu_stale_ticks++;
        if ((float)est->imu_stale_ticks * dt > est->cfg.imu_max_stale_s)
            est->q_valid = false;
        return;
    }
    est->imu_stale_ticks = 0;

    const float alpha = est->baro_alpha;

    for (int i = 0; i < 3; i++) {
        if (!isfinite(est->vel[i])) est->vel[i] = est->last_good_vel[i];
        if (!isfinite(est->pos[i])) est->pos[i] = est->last_good_pos[i];
    }

    float raw_ax = isfinite(in->accel_mps2[0]) ? in->accel_mps2[0] : est->last_good_acc_original[0];
    float raw_ay = isfinite(in->accel_mps2[1]) ? in->accel_mps2[1] : est->last_good_acc_original[1];
    float raw_az = isfinite(in->accel_mps2[2]) ? in->accel_mps2[2] : est->last_good_acc_original[2];

    float gx = isfinite(in->gyro_rads[0]) ? in->gyro_rads[0] : est->prev_gyro[0];
    float gy = isfinite(in->gyro_rads[1]) ? in->gyro_rads[1] : est->prev_gyro[1];
    float gz = isfinite(in->gyro_rads[2]) ? in->gyro_rads[2] : est->prev_gyro[2];

    float baro_alt = in->baro_alt_m;
    if (isfinite(baro_alt)) {
        est->last_good_baro_alt       = baro_alt;
        est->last_good_baro_alt_valid = true;
    } else {
        baro_alt = est->last_good_baro_alt_valid ? est->last_good_baro_alt : est->pos[2];
    }

    float raw_a[3] = {raw_ax, raw_ay, raw_az};
    for (int i = 0; i < 3; i++) {
        est->buf[i][est->buf_idx] = raw_a[i];
        est->last_good_acc_original[i] = raw_a[i];
    }
    if (++est->buf_idx >= n_avg) est->buf_idx = 0;
    if (est->buf_count < n_avg) est->buf_count++;

    float avg[3] = {0};
    float inv_count = 1.0f / (float)est->buf_count;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < est->buf_count; j++) avg[i] += est->buf[i][j];
        avg[i] *= inv_count;
    }
    est->acc_original[0] = avg[0];
    est->acc_original[1] = avg[1];
    est->acc_original[2] = avg[2];

    if (in->collecting_calib)
        calib_push(est, avg);

    if (!est->q_valid) {
        /* Seeding reads the accelerometer as gravity, so it is gated with the
         * corrections: an IMU stall that clears mid-coast would otherwise
         * rebuild the attitude out of the drag vector and then call it valid.
         *
         * That leaves nothing to rebuild from in flight, so validity instead
         * comes back from the quaternion already held. It was never corrupted -
         * a stall stops it being advanced, it does not corrupt it - and the cost
         * is the drift accumulated over the stall. The alternative is an
         * unrecoverable loss of attitude for the rest of the flight, which the
         * roll controller has no answer to at all. */
        if (in->corrections_inhibited) {
            if (quat_finite(est->q))
                est->q_valid = true;
            else if (quat_finite(est->last_good_q)) {
                est->q       = est->last_good_q;
                est->q_valid = true;
            }
        }

        bool can_seed = !in->corrections_inhibited &&
                        (est->calib.done ||
                         (!est->cfg.require_calib && est->buf_count == n_avg));
        if (can_seed) {
            const float *grav = est->calib.done ? est->calib.mean : avg;
            quat_t q0 = quat_from_gravity(grav);
            if (quat_finite(q0)) {
                est->q           = q0;
                est->last_good_q = q0;
                est->q_valid     = true;
            }
        }
    }

    mahony_update(&est->mahony, &est->q, &est->last_good_q, est->q_valid,
                  avg, gx, gy, gz, in->mag_ut, in->mag_valid,
                  est->calib.done, in->corrections_inhibited, dt);

    float accel_mag = sqrtf(avg[0]*avg[0] + avg[1]*avg[1] + avg[2]*avg[2]);
    bool  accel_trustworthy = (accel_mag >= ACCEL_TRUST_MIN &&
                               accel_mag <= ACCEL_TRUST_MAX);

    float accel_world[3] = {0};
    quat_rotate(est->q, avg, accel_world);

    float grav_to_remove = est->calib.done ? est->calib.mag_mean : GRAVITY;
    accel_world[2] -= grav_to_remove;
    for (int i = 0; i < 3; i++)
        accel_world[i] -= est->acc_bias_w[i];

    const float MAX_ACCEL = 200.0f * 9.81f;
    for (int i = 0; i < 3; i++) {
        est->acc[i] = clampf(accel_world[i], -MAX_ACCEL, MAX_ACCEL);
        est->last_good_acc[i] = est->acc[i];
    }

    if (in->zupt_active && est->q_valid && accel_trustworthy) {
        for (int i = 0; i < 3; i++)
            est->acc_bias_w[i] += Ki_acc * est->acc[i];
        for (int i = 0; i < 3; i++)
            est->acc_bias_w[i] = clampf(est->acc_bias_w[i], -1.0f, 1.0f);
    }

    if (!est->baro_inited && in->baro_valid) {
        est->pos[2]           = baro_alt;
        est->last_good_pos[2] = baro_alt;
        est->prev_baro_alt    = baro_alt;
        est->baro_inited      = true;
    }

    for (int i = 0; i < 3; i++) {
        est->vel[i]          += est->acc[i] * dt;
        est->last_good_vel[i] = est->vel[i];
    }

    if (in->zupt_active) {
        est->vel[0] = 0.0f;
        est->vel[1] = 0.0f;
        est->pos[0] = 0.0f;
        est->pos[1] = 0.0f;
        est->last_good_vel[0] = est->last_good_vel[1] = 0.0f;
        est->last_good_pos[0] = est->last_good_pos[1] = 0.0f;
    }

    if (est->baro_inited && in->baro_vel_valid && isfinite(in->baro_vel_z)) {
        est->vel[2] = (1.0f - BARO_VEL_ALPHA) * est->vel[2]
                    + BARO_VEL_ALPHA          * in->baro_vel_z;
        est->last_good_vel[2] = est->vel[2];
    }

    est->pos[0] += est->vel[0] * dt;
    est->pos[1] += est->vel[1] * dt;
    est->last_good_pos[0] = est->pos[0];
    est->last_good_pos[1] = est->pos[1];

    float imu_pos_z = est->pos[2] + est->vel[2] * dt;
    est->pos[2] = (est->baro_inited && in->baro_valid)
                ? alpha * imu_pos_z + (1.0f - alpha) * baro_alt
                : imu_pos_z;
    est->last_good_pos[2] = est->pos[2];

    est->ang_vel[0] = gx; est->ang_vel[1] = gy; est->ang_vel[2] = gz;
    est->prev_gyro[0] = gx; est->prev_gyro[1] = gy; est->prev_gyro[2] = gz;
}

void attitude_euler(const attitude_t *est, float out[3])
{
    out[0] = atan2f(2.0f*(est->q.w*est->q.x + est->q.y*est->q.z),
                    1.0f - 2.0f*(est->q.x*est->q.x + est->q.y*est->q.y));
    out[1] = asinf(clampf(2.0f*(est->q.w*est->q.y - est->q.z*est->q.x), -1.0f, 1.0f));
    out[2] = atan2f(2.0f*(est->q.w*est->q.z + est->q.x*est->q.y),
                    1.0f - 2.0f*(est->q.y*est->q.y + est->q.z*est->q.z));
}
