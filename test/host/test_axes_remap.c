// Sensor-to-body axis remap: validation and application.
#include <assert.h>
#include <stdio.h>

#include "control/axes.h"
#include "utils/axes_remap.h"

static void test_identity_is_a_rotation(void)
{
    axes_remap_t r = AXES_REMAP_IDENTITY;
    assert(axes_remap_is_rotation(&r));
}

static void test_z_only_flip_is_rejected(void)
{
    axes_remap_t r = { .map = {0, 1, 2}, .sign = {1, 1, -1} };
    assert(!axes_remap_is_rotation(&r));
}

static void test_proper_180_is_accepted(void)
{
    axes_remap_t about_x = { .map = {0, 1, 2}, .sign = {1, -1, -1} };
    axes_remap_t about_y = { .map = {0, 1, 2}, .sign = {-1, 1, -1} };
    axes_remap_t about_z = { .map = {0, 1, 2}, .sign = {-1, -1, 1} };
    assert(axes_remap_is_rotation(&about_x));
    assert(axes_remap_is_rotation(&about_y));
    assert(axes_remap_is_rotation(&about_z));
}

static void test_quarter_turns_in_plane_are_accepted(void)
{
    axes_remap_t cw  = { .map = {1, 0, 2}, .sign = {1, -1, 1} };
    axes_remap_t ccw = { .map = {1, 0, 2}, .sign = {-1, 1, 1} };
    assert(axes_remap_is_rotation(&cw));
    assert(axes_remap_is_rotation(&ccw));
}

/* Swapping a pair of axes and leaving the signs alone is the mistake a
 * hand-written remap actually makes, and its determinant is -1. */
static void test_swap_without_sign_is_rejected(void)
{
    axes_remap_t xy = { .map = {1, 0, 2}, .sign = {1, 1, 1} };
    axes_remap_t yz = { .map = {0, 2, 1}, .sign = {1, 1, 1} };
    axes_remap_t xz = { .map = {2, 1, 0}, .sign = {1, 1, 1} };
    assert(!axes_remap_is_rotation(&xy));
    assert(!axes_remap_is_rotation(&yz));
    assert(!axes_remap_is_rotation(&xz));
}

/* The two 3-cycles are even, so they are rotations with all signs positive. */
static void test_cyclic_permutations_are_accepted(void)
{
    axes_remap_t fwd = { .map = {1, 2, 0}, .sign = {1, 1, 1} };
    axes_remap_t rev = { .map = {2, 0, 1}, .sign = {1, 1, 1} };
    assert(axes_remap_is_rotation(&fwd));
    assert(axes_remap_is_rotation(&rev));
}

static void test_unset_is_rejected(void)
{
    axes_remap_t r = AXES_REMAP_UNSET;
    assert(!axes_remap_is_rotation(&r));
}

static void test_malformed_remaps_are_rejected(void)
{
    axes_remap_t repeated   = { .map = {0, 0, 2}, .sign = {1, 1, 1} };
    axes_remap_t out_of_rng = { .map = {0, 1, 3}, .sign = {1, 1, 1} };
    axes_remap_t negative   = { .map = {0, 1, -1}, .sign = {1, 1, 1} };
    axes_remap_t bad_sign   = { .map = {0, 1, 2}, .sign = {1, 2, 1} };
    axes_remap_t zero_sign  = { .map = {0, 1, 2}, .sign = {1, 0, 1} };
    assert(!axes_remap_is_rotation(&repeated));
    assert(!axes_remap_is_rotation(&out_of_rng));
    assert(!axes_remap_is_rotation(&negative));
    assert(!axes_remap_is_rotation(&bad_sign));
    assert(!axes_remap_is_rotation(&zero_sign));
}

static void test_board_remaps_are_rotations(void)
{
    axes_remap_t imu = BOARD_IMU_REMAP;
    axes_remap_t mag = BOARD_MAG_REMAP;
    assert(axes_remap_is_rotation(&imu));
    assert(axes_remap_is_rotation(&mag));
}

/* Both packages sit on the board face that looks at the nose, and their
 * datasheet Z axes run opposite ways there, so exactly one of the two remaps
 * has to invert Z. */
static void test_board_remaps_disagree_on_z(void)
{
    axes_remap_t imu = BOARD_IMU_REMAP;
    axes_remap_t mag = BOARD_MAG_REMAP;
    assert(imu.map[2] == 2 && mag.map[2] == 2);
    assert(imu.sign[2] != mag.sign[2]);
}

static void test_apply_permutes_and_signs(void)
{
    axes_remap_t r = { .map = {1, 0, 2}, .sign = {1, -1, -1} };
    float in[3] = {1.0f, 2.0f, 3.0f};
    float out[3] = {0};

    axes_remap_apply(&r, in, out);
    assert(out[0] == 2.0f && out[1] == -1.0f && out[2] == -3.0f);

    float alias[3] = {1.0f, 2.0f, 3.0f};
    axes_remap_apply(&r, alias, alias);
    assert(alias[0] == 2.0f && alias[1] == -1.0f && alias[2] == -3.0f);
}

int main(void)
{
    test_identity_is_a_rotation();
    test_z_only_flip_is_rejected();
    test_proper_180_is_accepted();
    test_quarter_turns_in_plane_are_accepted();
    test_swap_without_sign_is_rejected();
    test_cyclic_permutations_are_accepted();
    test_unset_is_rejected();
    test_malformed_remaps_are_rejected();
    test_board_remaps_are_rotations();
    test_board_remaps_disagree_on_z();
    test_apply_permutes_and_signs();
    printf("axes_remap: OK\n");
    return 0;
}
