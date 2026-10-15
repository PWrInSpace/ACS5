#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "control/flight_sm.h"

/* Decides when the roll controller may drive the canards.
 *
 * Burnout is the trigger, not altitude. The maneuver is only executable while
 * there is dynamic pressure to work with, which on this airframe means the first
 * seconds of a 7.3 s coast, so anything that delays arming comes straight out of
 * the window. FLIGHT_STATE_FREE_FLIGHT already requires launch detection and
 * burnout detection, each debounced, so it is a far stronger statement about not
 * being on the pad than any altitude threshold.
 *
 * Altitude survives only as a floor: without it a state machine fooled on the
 * ground could unlock and drive four servos against a locked mechanism.
 *
 * Arming is one-way per flight. Dropping back to disarmed clears the whole
 * controller state, which would discard a latched abort and capture a fresh
 * reference - another 90 deg from wherever the airframe then points. */
typedef enum
{
    CTRL_ARM_PREARM = 0,
    CTRL_ARM_ARMED  = 1,
    CTRL_ARM_SPENT  = 2,
} ctrl_arm_state_t;

typedef struct
{
    uint32_t lock_travel_ticks; /* canard lock needs this long to clear before
                                   the servos may push against it */
    float    min_altitude_m;    /* AGL floor; a ground interlock, not a trigger */
} ctrl_arm_config_t;

ctrl_arm_config_t ctrl_arm_config_default(void);

typedef struct
{
    flight_state_t flight_state;
    float          alt_m;          /* AGL, meaningless without the datum */
    bool           baro_datum_set; /* ground reference captured */
} ctrl_arm_input_t;

typedef struct
{
    ctrl_arm_config_t cfg;
    ctrl_arm_state_t  state;
    uint32_t          free_flight_ticks;
} ctrl_arm_t;

void ctrl_arm_init(ctrl_arm_t *a, const ctrl_arm_config_t *cfg);

/* One control tick. Returns whether the canards may be driven this tick. */
bool ctrl_arm_step(ctrl_arm_t *a, const ctrl_arm_input_t *in);
