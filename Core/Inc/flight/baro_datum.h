/* Pad pressure datum: the barometer reads metres above the launch site. */
#pragma once

#include <stdbool.h>

typedef struct
{
    bool set;   /* a pad datum has been applied at least once */
} baro_datum_t;

typedef struct
{
    bool retarget_qnh;  /* apply this sample's pressure as the new QNH */
    bool publish;       /* sample's altitude is relative to the current datum */
} baro_datum_action_t;

/* Decide what to do with a fresh barometer sample; on_pad = STANDBY. */
baro_datum_action_t baro_datum_on_sample(baro_datum_t *d, bool on_pad);
