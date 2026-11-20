#include "filter/attitude.h"

#include "utils/control_rate.h"
#include "utils/mathx.h"

#include <math.h>

#define Ki_acc          0.00002f
#define BARO_VEL_ALPHA  0.02f

attitude_config_t attitude_config_default(void)
{
    return (attitude_config_t){
        .dt_s            = CONTROL_DT_S,
        .baro_tau_s      = 0.5f,
        .imu_max_stale_s = 0.1f,
        .n_avg           = ATTITUDE_N_AVG,
        .n_calib         = (int)SEC_TO_TICKS(20.0f),
        .require_calib   = true,
        .gyro_bias_tau_s     = 5.0f,
        .still_rate_max_rads = 5.0f * DEG2RAD,
        .still_tilt_rad      = 0.1f * DEG2RAD,
        .still_mag_tol       = 0.05f,
        .still_time_s        = 3.0f,
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
    est->still_need_ticks = (uint32_t)(est->cfg.still_time_s / est->cfg.dt_s + 0.5f);
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
    /* Preserve quaternion, calib, gyro bias and mag reference (soft standby reset). */
}

/* True once the pad has been continuously still for still_time_s. Gravity
 * direction catches slow tilts below any rate gate; the rate gate catches
 * rotation about the vertical, which gravity cannot see. */
static bool pad_is_still(attitude_t *est, const float gyro[3], const float acc[3])
{
    const attitude_config_t *c = &est->cfg;
    const float g_ref = est->calib.done ? est->calib.mag_mean : GRAVITY;
    const float mag   = sqrtf(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);

    bool quiet = mag > 0.0f && fabsf(mag - g_ref) <= c->still_mag_tol * g_ref;
    for (int i = 0; i < 3; i++)
        quiet = quiet && fabsf(gyro[i]) <= c->still_rate_max_rads;

    float u[3] = {0};
    if (mag > 0.0f)
        for (int i = 0; i < 3; i++) u[i] = acc[i] / mag;

    if (quiet && est->still_anchor_valid) {
        const float *a = est->still_anchor;
        const float cx = u[1]*a[2] - u[2]*a[1];
        const float cy = u[2]*a[0] - u[0]*a[2];
        const float cz = u[0]*a[1] - u[1]*a[0];
        quiet = (cx*cx + cy*cy + cz*cz) <= sinf(c->still_tilt_rad) * sinf(c->still_tilt_rad);
    }

    if (!quiet || !est->still_anchor_valid) {
        for (int i = 0; i < 3; i++) est->still_anchor[i] = u[i];
        est->still_anchor_valid = mag > 0.0f;
        est->still_ticks = 0;
        return false;
    }
    if (est->still_ticks < UINT32_MAX) est->still_ticks++;
    return est->still_ticks >= est->still_need_ticks;
}

/* Running mean while few samples exist, then an EMA with gyro_bias_tau_s. */
static void gyro_bias_update(attitude_t *est, const attitude_input_t *in, const float acc[3])
{
    if (!in->zupt_active) return;
    if (!pad_is_still(est, in->gyro_rads, acc)) return;

    if (est->gyro_bias_n < UINT32_MAX) est->gyro_bias_n++;
    float alpha = fmaxf(1.0f / (float)est->gyro_bias_n,
                        est->cfg.dt_s / fmaxf(est->cfg.gyro_bias_tau_s, est->cfg.dt_s));
    for (int i = 0; i < 3; i++)
        est->gyro_bias[i] += alpha * (in->gyro_rads[i] - est->gyro_bias[i]);
}

void attitude_update(attitude_t *est, const attitude_input_t *in)
{
    const float dt = est->cfg.dt_s;
    const int n_avg = est->cfg.n_avg;

    if (!isfinite(dt) || dt <= 0.0001f) return;

    if (!in->imu_valid) {
        est->rate_fresh = false;
        est->imu_stale_ticks++;
        if ((float)est->imu_stale_ticks * dt > est->cfg.imu_max_stale_s)
            est->q_valid = false;
        return;
    }
    est->imu_stale_ticks = 0;

    const float alpha = est->baro_alpha;

    float raw_ax = isfinite(in->accel_mps2[0]) ? in->accel_mps2[0] : est->last_good_acc_original[0];
    float raw_ay = isfinite(in->accel_mps2[1]) ? in->accel_mps2[1] : est->last_good_acc_original[1];
    float raw_az = isfinite(in->accel_mps2[2]) ? in->accel_mps2[2] : est->last_good_acc_original[2];

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

    float gx, gy, gz;
    est->rate_fresh = isfinite(in->gyro_rads[0]) && isfinite(in->gyro_rads[1]) &&
                      isfinite(in->gyro_rads[2]);
    if (est->rate_fresh) {
        gyro_bias_update(est, in, avg);
        gx = in->gyro_rads[0] - est->gyro_bias[0];
        gy = in->gyro_rads[1] - est->gyro_bias[1];
        gz = in->gyro_rads[2] - est->gyro_bias[2];
    } else {
        gx = est->prev_gyro[0]; gy = est->prev_gyro[1]; gz = est->prev_gyro[2];
    }

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
    for (int i = 0; i < 3; i++)
        est->acc[i] = clampf(accel_world[i], -MAX_ACCEL, MAX_ACCEL);

    if (in->zupt_active && est->q_valid && accel_trustworthy) {
        for (int i = 0; i < 3; i++)
            est->acc_bias_w[i] += Ki_acc * est->acc[i];
        for (int i = 0; i < 3; i++)
            est->acc_bias_w[i] = clampf(est->acc_bias_w[i], -1.0f, 1.0f);
    }

    if (!est->baro_inited && in->baro_valid) {
        est->pos[2]      = baro_alt;
        est->baro_inited = true;
    }

    for (int i = 0; i < 3; i++)
        est->vel[i] += est->acc[i] * dt;

    if (in->zupt_active) {
        est->vel[0] = 0.0f;
        est->vel[1] = 0.0f;
        est->pos[0] = 0.0f;
        est->pos[1] = 0.0f;
    }

    if (est->baro_inited && in->baro_vel_valid && isfinite(in->baro_vel_z)) {
        est->vel[2] = (1.0f - BARO_VEL_ALPHA) * est->vel[2]
                    + BARO_VEL_ALPHA          * in->baro_vel_z;
    }

    est->pos[0] += est->vel[0] * dt;
    est->pos[1] += est->vel[1] * dt;

    float imu_pos_z = est->pos[2] + est->vel[2] * dt;
    est->pos[2] = (est->baro_inited && in->baro_valid)
                ? alpha * imu_pos_z + (1.0f - alpha) * baro_alt
                : imu_pos_z;

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
