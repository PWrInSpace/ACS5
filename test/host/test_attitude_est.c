// Sanity anchors for the extracted complementary filter: gravity alignment,
// gyro integration, NaN hardening. Tolerances are loose - this guards the
// extraction, not filter quality.
#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "filter/attitude.h"
#include "control/control_rate.h"
#include "utils/mathx.h"

/* Tick counts below encode durations. */
#define EST_HZ ((int)CONTROL_LOOP_HZ)

static attitude_input_t level_input(void)
{
    attitude_input_t in = {0};
    /* Body frame, as the drivers deliver it: +Z runs to the nose, so an upright
     * airframe reads +g there. */
    in.accel_mps2[2] = GRAVITY;
    in.imu_valid     = true;
    return in;
}

static void test_gravity_init_level(void)
{
    attitude_config_t cfg = attitude_config_default();
    cfg.require_calib = false;
    cfg.n_avg = 4;
    attitude_t est;
    attitude_init(&est, &cfg);
    assert(!est.q_valid);

    attitude_input_t in = level_input();
    for (int i = 0; i < 8; i++) attitude_update(&est, &in);

    assert(est.q_valid);
    float euler[3];
    attitude_euler(&est, euler);
    assert(fabsf(euler[0]) < 0.05f);   /* roll ~0 */
    assert(fabsf(euler[1]) < 0.05f);   /* pitch ~0 */
    /* An inverted Z reaches the same two Euler angles through a 180 deg flip. */
    assert(est.q.w > 0.99f);
}

/* Powered flight and coast are the phases where the accelerometer is not
 * measuring gravity. On this airframe the coast reads a few tenths of a g of
 * drag, pointing aft rather than down; levelling against it drives the estimate
 * toward inverted, and the roll maneuver reads its angle off that same
 * quaternion. The magnitude window happens to exclude drag today, but that
 * margin is a number that moves with mass and drag coefficient, and it is the
 * only thing standing between a correct estimate and a tumbling one. The phase
 * has to decide, not the magnitude.
 *
 * Seeding is gated with the corrections for the same reason: a stalled IMU that
 * clears mid-coast would otherwise re-seed the attitude from the drag vector
 * and then report it valid. */
static void test_inhibited_corrections_leave_the_estimate_to_the_gyro(void)
{
    attitude_config_t cfg = attitude_config_default();
    cfg.require_calib = false;
    cfg.n_avg = 4;
    attitude_t est;
    attitude_init(&est, &cfg);

    attitude_input_t in = level_input();
    for (int i = 0; i < 8; i++) attitude_update(&est, &in);
    assert(est.q_valid);

    float before[3];
    attitude_euler(&est, before);

    /* A full 1 g tilted 30 deg off body Z: inside the trust window, so only the
     * phase gate can stop it being taken for gravity. Gyro stays at zero, so
     * anything that moves the estimate came from the accelerometer. */
    in.accel_mps2[0]        = GRAVITY * 0.5f;
    in.accel_mps2[2]        = GRAVITY * 0.866f;
    in.corrections_inhibited = true;
    for (int i = 0; i < EST_HZ * 2; i++) attitude_update(&est, &in);

    float after[3];
    attitude_euler(&est, after);
    assert(fabsf(after[0] - before[0]) < 0.01f);
    assert(fabsf(after[1] - before[1]) < 0.01f);
}

static void test_gyro_integration_90deg(void)
{
    attitude_config_t cfg = attitude_config_default();
    cfg.require_calib = false;
    cfg.n_avg = 4;
    attitude_t est;
    attitude_init(&est, &cfg);

    attitude_input_t in = level_input();
    for (int i = 0; i < 8; i++) attitude_update(&est, &in);

    /* Spin about body Z at 90 deg/s for exactly 1 s. */
    in.gyro_rads[2] = PI_F / 2.0f;
    for (int i = 0; i < EST_HZ; i++) attitude_update(&est, &in);

    float euler[3];
    attitude_euler(&est, euler);
    assert(fabsf(euler[2] - PI_F / 2.0f) < 0.35f);
    assert(fabsf(est.ang_vel[2] - PI_F / 2.0f) < 1e-4f);
}

static void test_nan_inputs_do_not_poison(void)
{
    attitude_config_t cfg = attitude_config_default();
    cfg.require_calib = false;
    cfg.n_avg = 4;
    attitude_t est;
    attitude_init(&est, &cfg);

    attitude_input_t in = level_input();
    for (int i = 0; i < 8; i++) attitude_update(&est, &in);

    in.gyro_rads[0] = NAN;
    in.accel_mps2[1] = NAN;
    for (int i = 0; i < 100; i++) attitude_update(&est, &in);

    assert(isfinite(est.q.w) && isfinite(est.pos[2]) && isfinite(est.vel[2]));
}

static void test_reset_clears_kinematics(void)
{
    attitude_config_t cfg = attitude_config_default();
    cfg.require_calib = false;
    cfg.n_avg = 4;
    attitude_t est;
    attitude_init(&est, &cfg);

    attitude_input_t in = level_input();
    in.baro_alt_m = 120.0f;
    in.baro_valid = true;
    for (int i = 0; i < 200; i++) attitude_update(&est, &in);

    est.vel[2] = 5.0f;                    /* fake accumulated drift */
    assert(est.q_valid);
    attitude_reset(&est);
    assert(est.vel[2] == 0.0f);
    assert(fabsf(est.pos[2] - 120.0f) < 1.0f);   /* reseeded from baro */
    assert(est.q_valid);                  /* soft reset preserves attitude */
}

static void test_imu_dropout_drops_q_valid(void)
{
    attitude_config_t cfg = attitude_config_default();
    cfg.require_calib = false;
    cfg.n_avg = 4;
    attitude_t est;
    attitude_init(&est, &cfg);

    attitude_input_t in = level_input();
    for (int i = 0; i < 10; i++) attitude_update(&est, &in);
    assert(est.q_valid);

    /* Short dropout: state held, attitude still trusted. */
    in.imu_valid = false;
    quat_t q_before = est.q;
    for (int i = 0; i < 10; i++) attitude_update(&est, &in);
    assert(est.q_valid);
    assert(est.q.w == q_before.w && est.q.x == q_before.x);

    /* Past imu_max_stale_s the attitude is declared lost. */
    int limit_ticks = (int)(cfg.imu_max_stale_s / cfg.dt_s);
    for (int i = 0; i < limit_ticks; i++) attitude_update(&est, &in);
    assert(!est.q_valid);
}

/* An IMU stall long enough to invalidate the attitude must not be permanent in
 * flight. Re-validation runs through gravity seeding, and seeding is gated off
 * exactly when the accelerometer is measuring drag rather than gravity - so
 * without a separate path a single stall costs the attitude for the rest of the
 * coast, and the roll controller spends it in abort with nothing to steer by.
 * The quaternion was never corrupted, only left un-advanced, so validity comes
 * back from the propagated attitude, drift and all. */
static void test_imu_stall_in_flight_recovers_validity(void)
{
    attitude_config_t cfg = attitude_config_default();
    cfg.require_calib = false;
    cfg.n_avg = 4;
    attitude_t est;
    attitude_init(&est, &cfg);

    attitude_input_t in = level_input();
    for (int i = 0; i < 10; i++) attitude_update(&est, &in);
    assert(est.q_valid);
    quat_t before = est.q;

    /* Coast: corrections inhibited, then the IMU stops answering. */
    in.corrections_inhibited = true;
    in.imu_valid = false;
    int limit_ticks = (int)(cfg.imu_max_stale_s / cfg.dt_s) + 10;
    for (int i = 0; i < limit_ticks; i++) attitude_update(&est, &in);
    assert(!est.q_valid);

    in.imu_valid = true;
    attitude_update(&est, &in);

    assert(est.q_valid);
    /* Resumed from where it was, not rebuilt out of the drag vector. */
    assert(fabsf(est.q.w - before.w) < 0.01f);
    assert(fabsf(est.q.z - before.z) < 0.01f);
}

/* ---- truth model: propagate an attitude and hand back the sensors it implies ---- */

static quat_t truth_step(quat_t q, const float w_body[3], float dt)
{
    quat_t d = quat_mul(q, (quat_t){0.0f, w_body[0], w_body[1], w_body[2]});
    return quat_normalize((quat_t){q.w + 0.5f*d.w*dt, q.x + 0.5f*d.x*dt,
                                   q.y + 0.5f*d.y*dt, q.z + 0.5f*d.z*dt});
}

static void body_gravity(quat_t q, float out[3])
{
    float g_w[3] = {0.0f, 0.0f, GRAVITY};
    quat_rotate(quat_conj(q), g_w, out);
}

/* ~50 uT at ~66 deg inclination, which is central Europe. */
static void body_field(quat_t q, float out[3])
{
    float m_w[3] = {14.4f, 14.4f, -45.7f};
    quat_rotate(quat_conj(q), m_w, out);
}

static float attitude_error_deg(quat_t est, quat_t truth)
{
    quat_t e = quat_mul(quat_conj(est), truth);
    return 2.0f * acosf(clampf(fabsf(e.w), -1.0f, 1.0f)) * RAD2DEG;
}

static void seed_level(attitude_t *est, attitude_input_t *in, attitude_config_t *cfg)
{
    *in = level_input();
    in->collecting_calib = true;
    for (int i = 0; i < cfg->n_calib + 4; i++) attitude_update(est, in);
    in->collecting_calib = false;
    assert(est->q_valid);
    assert(est->calib.done);
}

static attitude_config_t bench_config(void)
{
    attitude_config_t cfg = attitude_config_default();
    cfg.require_calib = false;
    cfg.n_avg   = 4;
    cfg.n_calib = 8;
    return cfg;
}

/* Angle between estimate and truth after holding a constant body rate. bias_z is
 * added to the reported rate only, so the estimate drifts off a truth the accel
 * and mag still describe honestly. */
static float rotation_error_deg(const float w_body[3], float secs, bool mag_on,
                                float bias_z)
{
    attitude_config_t cfg = bench_config();
    attitude_t est;
    attitude_init(&est, &cfg);

    attitude_input_t in;
    seed_level(&est, &in, &cfg);
    in.mag_valid = mag_on;

    quat_t truth = quat_identity();
    int n = (int)(secs / cfg.dt_s);
    for (int i = 0; i < n; i++) {
        truth = truth_step(truth, w_body, cfg.dt_s);
        float a[3], m[3];
        body_gravity(truth, a);
        body_field(truth, m);
        for (int k = 0; k < 3; k++) {
            in.accel_mps2[k] = a[k];
            in.gyro_rads[k]  = w_body[k];
            in.mag_ut[k]     = m[k];
        }
        in.gyro_rads[2] += bias_z;
        attitude_update(&est, &in);
    }
    return attitude_error_deg(est.q, truth);
}

static void test_tilt_about_x_tracks_truth(void)
{
    float w[3] = {30.0f / RAD2DEG, 0.0f, 0.0f};
    assert(rotation_error_deg(w, 1.0f, false, 0.0f) < 2.0f);
}

static void test_tilt_about_y_tracks_truth(void)
{
    float w[3] = {0.0f, 30.0f / RAD2DEG, 0.0f};
    assert(rotation_error_deg(w, 1.0f, false, 0.0f) < 2.0f);
}

/* Rates on all three axes at once. A remap that levels correctly but is not a
 * rotation only shows up here, where the accelerometer and the gyro have to
 * agree about a motion that is not confined to one axis. */
static void test_combined_rotation_tracks_truth(void)
{
    float w[3] = {25.0f / RAD2DEG, -18.0f / RAD2DEG, 40.0f / RAD2DEG};
    assert(rotation_error_deg(w, 1.5f, false, 0.0f) < 3.0f);
}

/* The airframe really turns while the reported rate carries a bias, so the mag
 * term is non-zero throughout and its sign decides whether the error closes. */
static void test_mag_does_not_fight_real_rotation(void)
{
    float w[3] = {0.0f, 0.0f, 20.0f / RAD2DEG};
    assert(rotation_error_deg(w, 6.0f, true, 0.5f / RAD2DEG) < 3.0f);
}

static void test_mag_bounds_heading_drift(void)
{
    const float bias = 0.5f / RAD2DEG;
    float drift[2];

    for (int mag_on = 0; mag_on <= 1; mag_on++) {
        attitude_config_t cfg = bench_config();
        attitude_t est;
        attitude_init(&est, &cfg);

        attitude_input_t in;
        seed_level(&est, &in, &cfg);
        in.mag_valid = (mag_on != 0);

        float m[3];
        body_field(quat_identity(), m);
        for (int k = 0; k < 3; k++) in.mag_ut[k] = m[k];
        in.gyro_rads[2] = bias;

        for (int i = 0; i < (int)(30.0f / cfg.dt_s); i++) attitude_update(&est, &in);

        const float axis[3] = {0.0f, 0.0f, 1.0f};
        drift[mag_on] = fabsf(quat_twist_angle(est.q, axis)) * RAD2DEG;
    }

    assert(drift[1] < 5.0f);
    assert(drift[1] < drift[0]);
}

/* Corrections inhibited, as in flight: the gyro alone carries the attitude and
 * the twist the roll controller steers by has to survive off-axis motion. */
static void test_coning_does_not_leak_into_roll(void)
{
    attitude_config_t cfg = bench_config();
    attitude_t est;
    attitude_init(&est, &cfg);

    attitude_input_t in;
    seed_level(&est, &in, &cfg);
    in.corrections_inhibited = true;

    quat_t truth = quat_identity();
    for (int i = 0; i < (int)(4.0f / cfg.dt_s); i++) {
        float t = (float)i * cfg.dt_s;
        float w[3] = {0.35f * cosf(2.0f * t), 0.35f * sinf(2.0f * t), 1.5f};
        truth = truth_step(truth, w, cfg.dt_s);
        for (int k = 0; k < 3; k++) in.gyro_rads[k] = w[k];
        attitude_update(&est, &in);
    }

    const float axis[3] = {0.0f, 0.0f, 1.0f};
    float twist_est   = quat_twist_angle(est.q, axis) * RAD2DEG;
    float twist_truth = quat_twist_angle(truth, axis) * RAD2DEG;
    assert(fabsf(twist_est - twist_truth) < 2.0f);
}

int main(void)
{
    test_gravity_init_level();
    test_imu_stall_in_flight_recovers_validity();
    test_inhibited_corrections_leave_the_estimate_to_the_gyro();
    test_gyro_integration_90deg();
    test_tilt_about_x_tracks_truth();
    test_tilt_about_y_tracks_truth();
    test_combined_rotation_tracks_truth();
    test_mag_does_not_fight_real_rotation();
    test_mag_bounds_heading_drift();
    test_coning_does_not_leak_into_roll();
    test_nan_inputs_do_not_poison();
    test_reset_clears_kinematics();
    test_imu_dropout_drops_q_valid();
    printf("attitude: OK\n");
    return 0;
}
