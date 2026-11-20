// Phase-dependent estimator gates and barometer freshness.
#include "test_assert.h"

#include "flight/estimator_policy.h"

static void test_pad_trusts_gravity_and_clamps_drift(void)
{
    estimator_gates_t g = estimator_gates(FLIGHT_STATE_STANDBY, false);
    CHECK(!g.corrections_inhibited);
    CHECK(g.collecting_calib);
    CHECK(g.zupt_active);
    CHECK(!estimator_gates(FLIGHT_STATE_STANDBY, true).collecting_calib);
}

static void test_flight_phases_inhibit_corrections(void)
{
    const flight_state_t flying[] = {
        FLIGHT_STATE_ACCELERATION, FLIGHT_STATE_FREE_FLIGHT, FLIGHT_STATE_FALLING,
    };
    for (unsigned i = 0; i < sizeof flying / sizeof *flying; i++) {
        estimator_gates_t g = estimator_gates(flying[i], false);
        CHECK(g.corrections_inhibited);
        CHECK(!g.collecting_calib);
        CHECK(!g.zupt_active);
    }
}

static void test_recovery_restores_corrections(void)
{
    estimator_gates_t g = estimator_gates(FLIGHT_STATE_RECOVERY, true);
    CHECK(!g.corrections_inhibited);
    CHECK(!g.zupt_active);
}

static void test_baro_vel_needs_datum_on_pad(void)
{
    CHECK(!estimator_baro_vel_valid(FLIGHT_STATE_STANDBY, false));
    CHECK(estimator_baro_vel_valid(FLIGHT_STATE_STANDBY, true));
    CHECK(estimator_baro_vel_valid(FLIGHT_STATE_FREE_FLIGHT, false));
}

static void test_baro_freshness(void)
{
    CHECK(!baro_sample_fresh(0u, 50000u, 100000u));
    CHECK(baro_sample_fresh(1000u, 100999u, 100000u));
    CHECK(!baro_sample_fresh(1000u, 101000u, 100000u));
    CHECK(baro_sample_fresh(0xFFFFFF00u, 0x00000100u, 100000u));   /* counter wrap */
}

int main(void)
{
    test_pad_trusts_gravity_and_clamps_drift();
    test_flight_phases_inhibit_corrections();
    test_recovery_restores_corrections();
    test_baro_vel_needs_datum_on_pad();
    test_baro_freshness();
    return test_report("estimator_policy");
}
