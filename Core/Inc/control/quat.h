#pragma once

#include <math.h>
#include <stdbool.h>

/* Unit-quaternion helpers shared by the attitude estimator and the roll
 * controller. Convention: q = {w, x, y, z}, Hamilton product, q rotates
 * body-frame vectors into world frame. */
typedef struct { float w, x, y, z; } quat_t;

static inline bool quat_finite(quat_t q)
{
    return isfinite(q.w) && isfinite(q.x) && isfinite(q.y) && isfinite(q.z);
}

quat_t quat_identity(void);
quat_t quat_mul(quat_t a, quat_t b);
quat_t quat_conj(quat_t q);
quat_t quat_normalize(quat_t q);
void   quat_rotate(quat_t q, const float v[3], float out[3]);

/* Signed twist of q about the unit axis (swing-twist decomposition), wrapped to
 * (-pi, pi]. The twist is identical whether q is read as swing * twist or
 * twist * swing. axis must be unit; a zero/degenerate q reports 0. */
float quat_twist_angle(quat_t q, const float axis[3]);

/* Wrap an angle to (-pi, pi]. */
float quat_wrap_pi(float a);
