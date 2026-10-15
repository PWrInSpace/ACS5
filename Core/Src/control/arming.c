#include "control/arming.h"
#include "control/control_rate.h"

ctrl_arm_config_t ctrl_arm_config_default(void)
{
    return (ctrl_arm_config_t){
        .lock_travel_ticks = SEC_TO_TICKS(0.5f),
        /* Burnout is at about 148 m on this airframe. The margin is thinner than
         * that looks: this is raw barometric AGL on a vehicle at 77.6 m/s, where
         * port and boundary-layer effects read low and the sensor lags. Since a
         * delayed arm comes straight out of the dynamic-pressure budget the
         * maneuver depends on, the floor sits well under the worst credible
         * error while staying far above anything the pad can produce. It is not
         * a trigger and must not be tuned like one. */
        .min_altitude_m    = 50.0f,
    };
}

void ctrl_arm_init(ctrl_arm_t *a, const ctrl_arm_config_t *cfg)
{
    *a = (ctrl_arm_t){ .cfg = *cfg };
}

bool ctrl_arm_step(ctrl_arm_t *a, const ctrl_arm_input_t *in)
{
    const ctrl_arm_config_t *c = &a->cfg;

    /* Saturating, so a long coast cannot wrap the counter back under the
     * threshold. */
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
            /* Leaving free flight is apogee. Altitude is deliberately not
             * consulted here: the arming point sits within metres of the floor,
             * which is exactly where barometer noise crosses it, and a disarm
             * is permanent. */
            if (in->flight_state != FLIGHT_STATE_FREE_FLIGHT)
                a->state = CTRL_ARM_SPENT;
            break;

        case CTRL_ARM_SPENT:
            break;
    }

    return a->state == CTRL_ARM_ARMED;
}
