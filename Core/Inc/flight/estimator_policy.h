/* Which estimator aids are trusted in each flight phase. */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "flight/flight_sm.h"

typedef struct
{
    bool corrections_inhibited;  /* accel/mag do not measure gravity/heading */
    bool collecting_calib;       /* pad gravity calibration still running */
    bool zupt_active;            /* vehicle is on the pad */
} estimator_gates_t;

estimator_gates_t estimator_gates(flight_state_t state, bool calib_done);

/* True when the barometric vertical speed may aid the estimator. */
bool estimator_baro_vel_valid(flight_state_t state, bool baro_datum_set);

/* True when a sample stamped sample_us is younger than max_age_us at now_us; 0 = never sampled. */
bool baro_sample_fresh(uint32_t sample_us, uint32_t now_us, uint32_t max_age_us);
