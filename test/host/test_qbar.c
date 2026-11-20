// Dynamic pressure from barometric density and estimator velocity.
#include <math.h>

#include "test_assert.h"

#include "control/qbar.h"

static void test_sea_level_isa(void)
{
    const float v[3] = {0.0f, 0.0f, 100.0f};
    /* ISA sea level: rho = 1.2250 kg/m^3 -> q = 6125 Pa at 100 m/s. */
    CHECK_NEAR(qbar_from_baro(true, 101325.0f, 15.0f, v), 6125.0f, 2.0f);
}

static void test_uses_full_velocity_vector(void)
{
    const float v[3] = {30.0f, 40.0f, 0.0f};
    const float vz[3] = {0.0f, 0.0f, 50.0f};
    CHECK_NEAR(qbar_from_baro(true, 90000.0f, 5.0f, v),
               qbar_from_baro(true, 90000.0f, 5.0f, vz), 1e-3f);
}

static void test_unknown_inputs_give_zero(void)
{
    const float v[3] = {0.0f, 0.0f, 80.0f};
    const float v_nan[3] = {0.0f, 0.0f, NAN};
    CHECK(qbar_from_baro(false, 101325.0f, 15.0f, v) == 0.0f);
    CHECK(qbar_from_baro(true, NAN, 15.0f, v) == 0.0f);
    CHECK(qbar_from_baro(true, 101325.0f, INFINITY, v) == 0.0f);
    CHECK(qbar_from_baro(true, 101325.0f, 15.0f, v_nan) == 0.0f);
    CHECK(qbar_from_baro(true, -5.0f, 15.0f, v) == 0.0f);
}

int main(void)
{
    test_sea_level_isa();
    test_uses_full_velocity_vector();
    test_unknown_inputs_give_zero();
    return test_report("qbar");
}
