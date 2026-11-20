#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "flight/flight_sm.h"

/* Decides when the roll controller may drive the canards; arming is one-way per flight.
 * Why burnout, not altitude, is the trigger: docs/design.md#arming-trigger */
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
