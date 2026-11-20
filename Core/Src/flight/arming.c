#include "flight/arming.h"
#include "config/airframe.h"
#include "utils/control_rate.h"

ctrl_arm_config_t ctrl_arm_config_default(void)
{
    return (ctrl_arm_config_t){
        .lock_travel_ticks = SEC_TO_TICKS(AIRFRAME_CANARD_LOCK_TRAVEL_S),
        /* A ground interlock, not a trigger: docs/design.md#altitude-floor */
        .min_altitude_m    = AIRFRAME_ARM_MIN_ALTITUDE_M,
    };
}

void ctrl_arm_init(ctrl_arm_t *a, const ctrl_arm_config_t *cfg)
{
    *a = (ctrl_arm_t){ .cfg = *cfg };
}

bool ctrl_arm_step(ctrl_arm_t *a, const ctrl_arm_input_t *in)
{
    const ctrl_arm_config_t *c = &a->cfg;

    /* Saturating, so a long coast cannot wrap the counter back under the threshold. */
    if (in->flight_state == FLIGHT_STATE_FREE_FLIGHT) {
        if (a->free_flight_ticks < c->lock_travel_ticks)
            a->free_flight_ticks++;
    } else {
        a->free_flight_ticks = 0;
    }

    switch (a->state)
    {
        case CTRL_ARM_PREARM:
            if (in->flight_state == FLIGHT_STATE_FREE_FLIGHT &&
                a->free_flight_ticks >= c->lock_travel_ticks &&
                in->baro_datum_set &&
                in->alt_m >= c->min_altitude_m)
                a->state = CTRL_ARM_ARMED;
            break;

        case CTRL_ARM_ARMED:
            /* Leaving free flight is apogee. Altitude is not consulted: baro noise
             * crosses the floor near the arming point, and a disarm is permanent. */
            if (in->flight_state != FLIGHT_STATE_FREE_FLIGHT)
                a->state = CTRL_ARM_SPENT;
            break;

        case CTRL_ARM_SPENT:
            break;
    }

    return a->state == CTRL_ARM_ARMED;
}
