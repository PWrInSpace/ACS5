// Arming gate: when the roll controller is allowed to drive the canards.
// One-way per flight, so every one of these is a decision that cannot be undone
// in the air.
#include <assert.h>
#include <stdio.h>

#include "control/arming.h"
#include "control/control_rate.h"

/* One second of control ticks: longer than any gate under test. */
#define CTRL_LOOP_TICKS ((int)CONTROL_LOOP_HZ)

static ctrl_arm_config_t test_cfg(void)
{
    ctrl_arm_config_t c = ctrl_arm_config_default();
    c.min_altitude_m = 100.0f;
    return c;
}

/* Flies the gate forward n ticks in one flight state and returns the last
 * enable decision. */
static bool run(ctrl_arm_t *a, flight_state_t st, float alt, bool datum, int ticks)
{
    bool en = false;
    for (int i = 0; i < ticks; i++) {
        ctrl_arm_input_t in = { .flight_state = st, .alt_m = alt,
                                .baro_datum_set = datum };
        en = ctrl_arm_step(a, &in);
    }
    return en;
}

/* The pad is the expensive failure: four T75s stalling against a locked
 * mechanism. Altitude alone must never be enough, however wrong the barometer
 * is about where the ground is. */
static void test_never_arms_before_free_flight(void)
{
    ctrl_arm_config_t cfg = test_cfg();
    ctrl_arm_t a;
    ctrl_arm_init(&a, &cfg);

    assert(!run(&a, FLIGHT_STATE_STANDBY, 5000.0f, true, CTRL_LOOP_TICKS));
    assert(!run(&a, FLIGHT_STATE_ACCELERATION, 5000.0f, true, CTRL_LOOP_TICKS));
    assert(a.state == CTRL_ARM_PREARM);
}

/* The canard lock is a plain RC servo with no slew limit and about 200 ms of
 * travel. Commanding the canards into a mechanism that has not released yet
 * stalls them at 2.8 A each. */
static void test_waits_for_the_canard_lock_to_clear(void)
{
    ctrl_arm_config_t cfg = test_cfg();
    ctrl_arm_t a;
    ctrl_arm_init(&a, &cfg);

    assert(!run(&a, FLIGHT_STATE_FREE_FLIGHT, 200.0f, true,
                (int)cfg.lock_travel_ticks - 1));
    assert(run(&a, FLIGHT_STATE_FREE_FLIGHT, 200.0f, true, 2));
    assert(a.state == CTRL_ARM_ARMED);
}

/* Without the ground reference the altitude is height above the standard
 * pressure datum, not AGL, and the floor below means nothing. */
static void test_requires_a_baro_datum(void)
{
    ctrl_arm_config_t cfg = test_cfg();
    ctrl_arm_t a;
    ctrl_arm_init(&a, &cfg);

    assert(!run(&a, FLIGHT_STATE_FREE_FLIGHT, 200.0f, false, CTRL_LOOP_TICKS));
    assert(a.state == CTRL_ARM_PREARM);
}

static void test_respects_the_altitude_floor(void)
{
    ctrl_arm_config_t cfg = test_cfg();
    ctrl_arm_t a;
    ctrl_arm_init(&a, &cfg);

    assert(!run(&a, FLIGHT_STATE_FREE_FLIGHT, 50.0f, true, CTRL_LOOP_TICKS));
    assert(run(&a, FLIGHT_STATE_FREE_FLIGHT, 150.0f, true, 1));
}

/* The floor is a ground interlock, so it is checked once on the way in. Once
 * the vehicle is coasting, a barometer sample that dips below it is noise, and
 * treating that as a disarm would end the maneuver permanently - arming happens
 * within metres of the threshold, which is exactly where noise crosses it. */
static void test_altitude_noise_after_arming_does_not_disarm(void)
{
    ctrl_arm_config_t cfg = test_cfg();
    ctrl_arm_t a;
    ctrl_arm_init(&a, &cfg);

    assert(run(&a, FLIGHT_STATE_FREE_FLIGHT, 150.0f, true, CTRL_LOOP_TICKS));
    assert(run(&a, FLIGHT_STATE_FREE_FLIGHT, 20.0f, true, 1));
    assert(run(&a, FLIGHT_STATE_FREE_FLIGHT, 150.0f, true, 1));
    assert(a.state == CTRL_ARM_ARMED);
}

/* Apogee ends the maneuver, and the state machine already detects it with a
 * velocity gate plus an altitude-drop gate. Leaving FREE_FLIGHT is that event. */
static void test_disarms_at_apogee_and_never_rearms(void)
{
    ctrl_arm_config_t cfg = test_cfg();
    ctrl_arm_t a;
    ctrl_arm_init(&a, &cfg);

    assert(run(&a, FLIGHT_STATE_FREE_FLIGHT, 200.0f, true, CTRL_LOOP_TICKS));
    assert(!run(&a, FLIGHT_STATE_FALLING, 200.0f, true, 1));
    assert(a.state == CTRL_ARM_SPENT);

    /* Nothing brings it back, including a state machine that somehow reports
     * free flight again. */
    assert(!run(&a, FLIGHT_STATE_FREE_FLIGHT, 200.0f, true, CTRL_LOOP_TICKS));
    assert(a.state == CTRL_ARM_SPENT);
}

int main(void)
{
    test_never_arms_before_free_flight();
    test_waits_for_the_canard_lock_to_clear();
    test_requires_a_baro_datum();
    test_respects_the_altitude_floor();
    test_altitude_noise_after_arming_does_not_disarm();
    test_disarms_at_apogee_and_never_rearms();
    printf("arming: OK\n");
    return 0;
}
