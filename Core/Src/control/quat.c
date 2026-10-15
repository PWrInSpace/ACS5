#include "control/quat.h"
#include "utils/mathx.h"

#include <math.h>

quat_t quat_identity(void) { return (quat_t){1.0f, 0.0f, 0.0f, 0.0f}; }

quat_t quat_mul(quat_t a, quat_t b)
{
    return (quat_t){
        a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
        a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
    };
}

quat_t quat_conj(quat_t q) { return (quat_t){q.w, -q.x, -q.y, -q.z}; }

quat_t quat_normalize(quat_t q)
{
    float n = sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    if (n < 1e-6f) return quat_identity();
    float inv = 1.0f / n;
    return (quat_t){q.w*inv, q.x*inv, q.y*inv, q.z*inv};
}

void quat_rotate(quat_t q, const float v[3], float out[3])
{
    quat_t vq  = {0.0f, v[0], v[1], v[2]};
    quat_t res = quat_mul(quat_mul(q, vq), quat_conj(q));
    out[0] = res.x;
    out[1] = res.y;
    out[2] = res.z;
}

float quat_wrap_pi(float a)
{
    /* Both flight callers pass |a| <= 2*pi, where the two adjustments below
     * already complete the wrap; fmodf is only needed for arbitrary inputs. */
    if (fabsf(a) > 2.0f * PI_F) a = fmodf(a, 2.0f * PI_F);
    if (a >   PI_F) a -= 2.0f * PI_F;
    if (a <= -PI_F) a += 2.0f * PI_F;
    return a;
}

float quat_twist_angle(quat_t q, const float axis[3])
{
    float d = q.x*axis[0] + q.y*axis[1] + q.z*axis[2];
    float n = sqrtf(q.w*q.w + d*d);
    if (n < 1e-9f) return 0.0f;   /* zero/degenerate q: no twist to report */
    return quat_wrap_pi(2.0f * atan2f(d, q.w));
}
