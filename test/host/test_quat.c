// Quaternion math used by the attitude estimator and roll controller.
// Anchors: axis-angle constructions with known results.
#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "control/quat.h"
#include "utils/mathx.h"

static quat_t axis_angle(float x, float y, float z, float angle)
{
    float s = sinf(angle * 0.5f);
    return (quat_t){cosf(angle * 0.5f), x * s, y * s, z * s};
}

static int near(float a, float b, float tol) { return fabsf(a - b) < tol; }

static void test_mul_identity(void)
{
    quat_t q = axis_angle(0, 0, 1, 0.7f);
    quat_t r = quat_mul(quat_identity(), q);
    assert(near(r.w, q.w, 1e-6f) && near(r.z, q.z, 1e-6f));
}

static void test_rotate_90_about_z(void)
{
    quat_t q = axis_angle(0, 0, 1, PI_F / 2.0f);
    float v[3] = {1, 0, 0}, out[3];
    quat_rotate(q, v, out);
    assert(near(out[0], 0, 1e-5f) && near(out[1], 1, 1e-5f) && near(out[2], 0, 1e-5f));
}

static void test_conj_inverts_rotation(void)
{
    /* conj is the inverse only for |q| = 1, so the axis must be exactly unit:
     * (2/3, 1/3, 2/3) is, to float precision. */
    quat_t q = axis_angle(2.0f / 3.0f, 1.0f / 3.0f, 2.0f / 3.0f, 1.1f);
    quat_t qq = quat_mul(q, quat_conj(q));
    assert(near(qq.w, 1, 1e-5f) && near(qq.x, 0, 1e-5f));
}

static void test_twist_pure_roll(void)
{
    const float z_axis[3] = {0, 0, 1};
    assert(near(quat_twist_angle(axis_angle(0, 0, 1, 0.5f), z_axis), 0.5f, 1e-5f));
    assert(near(quat_twist_angle(axis_angle(0, 0, 1, -1.2f), z_axis), -1.2f, 1e-5f));
    assert(near(quat_twist_angle(quat_identity(), z_axis), 0.0f, 1e-6f));
}

static void test_twist_ignores_swing(void)
{
    const float z_axis[3] = {0, 0, 1};
    quat_t twist = axis_angle(0, 0, 1, 0.5f);
    quat_t swing = axis_angle(1, 0, 0, 0.3f);
    assert(near(quat_twist_angle(quat_mul(swing, twist), z_axis), 0.5f, 1e-4f));

    /* Even a 180 deg swing must not corrupt the twist - that is the attitude the
     * rocket reaches if it tumbles, and the roll angle must stay readable. */
    quat_t flipped = quat_mul(axis_angle(1, 0, 0, PI_F), twist);
    assert(near(quat_twist_angle(flipped, z_axis), 0.5f, 1e-4f));
}

static void test_twist_survives_double_cover(void)
{
    /* The estimator may emit -q for the same rotation. The roll controller
     * accumulates twist deltas, so a sign flip would corrupt the maneuver
     * angle permanently. */
    const float z_axis[3] = {0, 0, 1};
    quat_t q  = quat_mul(axis_angle(1, 0, 0, 0.3f), axis_angle(0, 0, 1, 0.5f));
    quat_t nq = {-q.w, -q.x, -q.y, -q.z};
    assert(near(quat_twist_angle(q, z_axis), quat_twist_angle(nq, z_axis), 1e-6f));
}

static void test_normalize(void)
{
    quat_t q = quat_normalize((quat_t){2.0f, 0.0f, 0.0f, 2.0f});
    float n = sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    assert(near(n, 1.0f, 1e-6f));
    assert(near(q.w, 0.7071068f, 1e-5f) && near(q.z, 0.7071068f, 1e-5f));

    /* Degenerate input must fall back to identity, not divide by ~0 - this is
     * the estimator's guard against a collapsed quaternion. */
    quat_t z = quat_normalize((quat_t){0.0f, 0.0f, 0.0f, 0.0f});
    assert(near(z.w, 1.0f, 1e-6f) && near(z.x, 0, 1e-6f) &&
           near(z.y, 0, 1e-6f) && near(z.z, 0, 1e-6f));
}

static void test_wrap_pi(void)
{
    assert(near(quat_wrap_pi(3.5f * PI_F), -0.5f * PI_F, 1e-4f));
    assert(near(quat_wrap_pi(-3.5f * PI_F), 0.5f * PI_F, 1e-4f));
    assert(near(quat_wrap_pi(0.3f), 0.3f, 1e-6f));

    /* Half-open range: both boundaries land on +pi. */
    assert(near(quat_wrap_pi(PI_F), PI_F, 1e-5f));
    assert(near(quat_wrap_pi(-PI_F), PI_F, 1e-5f));

    /* Arbitrarily large input must still terminate and land in range. */
    float big = quat_wrap_pi(1e9f);
    assert(big > -PI_F && big <= PI_F);
}

int main(void)
{
    test_mul_identity();
    test_rotate_90_about_z();
    test_conj_inverts_rotation();
    test_twist_pure_roll();
    test_twist_ignores_swing();
    test_twist_survives_double_cover();
    test_normalize();
    test_wrap_pi();
    printf("quat: OK\n");
    return 0;
}
