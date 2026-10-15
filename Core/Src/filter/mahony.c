#include "filter/mahony.h"

#include "utils/mathx.h"

#include <math.h>

#define Kp_accel        0.5f
#define Kp_mag          0.25f
#define Ki              0.005f
#define ACCEL_TRUST_MIN (0.7f * GRAVITY)
#define ACCEL_TRUST_MAX (1.3f * GRAVITY)

void mahony_init(mahony_t *m)
{
    *m = (mahony_t){0};
    m->ref_mag_h[0] = 1.0f;
    m->ref_mag_h[1] = 0.0f;
}

void mahony_reset_integral(mahony_t *m)
{
    m->integral[0] = m->integral[1] = m->integral[2] = 0.0f;
}

void mahony_update(mahony_t *m,
                   quat_t *q,
                   quat_t *last_good_q,
                   bool q_valid,
                   const float avg[3],
                   float gx, float gy, float gz,
                   const float mag_ut[3],
                   bool mag_valid,
                   bool calib_done,
                   bool corrections_inhibited,
                   float dt)
{
    float safe_dt = fmaxf(dt, 1e-6f);
    float ex = 0.0f, ey = 0.0f, ez = 0.0f;

    float accel_mag = sqrtf(avg[0]*avg[0] + avg[1]*avg[1] + avg[2]*avg[2]);
    /* The magnitude window alone cannot tell gravity from drag: it only says
     * the vector is about 1 g, and a decelerating airframe can produce that.
     * The flight phase is what actually knows, so it gates both corrections -
     * the magnitude check stays as a second line, for a sensor gone wrong
     * rather than a vehicle in the wrong phase. */
    bool  accel_trustworthy = !corrections_inhibited &&
                              (accel_mag >= ACCEL_TRUST_MIN &&
                               accel_mag <= ACCEL_TRUST_MAX);

    if (q_valid) {
        if (accel_trustworthy) {
            float vx = 2.0f*(q->x*q->z - q->w*q->y);
            float vy = 2.0f*(q->w*q->x + q->y*q->z);
            float vz = q->w*q->w - q->x*q->x
                     - q->y*q->y + q->z*q->z;

            float inv_mag = 1.0f / accel_mag;
            float ax_n = avg[0]*inv_mag, ay_n = avg[1]*inv_mag, az_n = avg[2]*inv_mag;

            ex += ay_n*vz - az_n*vy;
            ey += az_n*vx - ax_n*vz;
            ez += ax_n*vy - ay_n*vx;
        }

        if (accel_trustworthy && mag_valid) {
            float mx = mag_ut[0];
            float my = mag_ut[1];
            float mz = mag_ut[2];

            float mag_body_mag = sqrtf(mx*mx + my*my + mz*mz);
            bool  mag_ok = isfinite(mx) && isfinite(my) && isfinite(mz)
                           && mag_body_mag > 1.0f;

            if (mag_ok) {
                float m_b[3] = {mx, my, mz};
                float m_w[3] = {0};
                quat_rotate(*q, m_b, m_w);

                float hx = m_w[0], hy = m_w[1];
                float h_mag = sqrtf(hx*hx + hy*hy);

                if (h_mag > 0.5f) {
                    hx /= h_mag; hy /= h_mag;

                    if (!m->mag_ref_inited && calib_done) {
                        m->ref_mag_h[0] = hx;
                        m->ref_mag_h[1] = hy;
                        m->mag_ref_inited = true;
                    }

                    if (m->mag_ref_inited) {
                        /* measured x expected, the same order as the accel term
                         * above. Reversed, this term is positive feedback on
                         * heading and the estimate walks away from the truth. */
                        float yaw_err_world = hx*m->ref_mag_h[1] - hy*m->ref_mag_h[0];
                        float err_w[3] = {0.0f, 0.0f, yaw_err_world};
                        float err_b[3] = {0};
                        quat_rotate(quat_conj(*q), err_w, err_b);

                        ex += Kp_mag / Kp_accel * err_b[0];
                        ey += Kp_mag / Kp_accel * err_b[1];
                        ez += Kp_mag / Kp_accel * err_b[2];
                    }
                }
            }
        }

        m->integral[0] += Ki * ex * safe_dt;
        m->integral[1] += Ki * ey * safe_dt;
        m->integral[2] += Ki * ez * safe_dt;
        for (int i = 0; i < 3; i++)
            m->integral[i] = clampf(m->integral[i], -0.1f, 0.1f);
    }

    float gxc = gx + Kp_accel * ex + m->integral[0];
    float gyc = gy + Kp_accel * ey + m->integral[1];
    float gzc = gz + Kp_accel * ez + m->integral[2];

    if (!q_valid)
        return;

    quat_t q_dot = quat_mul(*q, (quat_t){0.0f, gxc, gyc, gzc});
    quat_t q_cand = {
        q->w + 0.5f * q_dot.w * dt,
        q->x + 0.5f * q_dot.x * dt,
        q->y + 0.5f * q_dot.y * dt,
        q->z + 0.5f * q_dot.z * dt
    };
    float q_norm_sq = q_cand.w*q_cand.w + q_cand.x*q_cand.x +
                      q_cand.y*q_cand.y + q_cand.z*q_cand.z;
    if (q_norm_sq > 1e-6f && isfinite(q_norm_sq)) {
        quat_t q_n = quat_normalize(q_cand);
        if (isfinite(q_n.w)) { *q = q_n; *last_good_q = *q; }
        else                 *q = *last_good_q;
    } else {
        *q = *last_good_q;
    }
}
