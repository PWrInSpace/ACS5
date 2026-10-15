#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Values are logged as uint8_t into the flight CSV; post-flight tooling reads
 * them numerically, so do not renumber. */
typedef enum
{
    FLIGHT_STATE_STANDBY      = 0,
    FLIGHT_STATE_ACCELERATION = 1,
    FLIGHT_STATE_FREE_FLIGHT  = 2,
    FLIGHT_STATE_FALLING      = 3,
    FLIGHT_STATE_RECOVERY     = 4,
} flight_state_t;

typedef struct
{
    float    launch_accel_mps2;   /* STANDBY->ACCEL when az exceeds this */
    int      launch_debounce;     /* consecutive ticks required */
    float    burnout_accel_mps2;  /* ACCEL->FREE_FLIGHT when az at/below this */
    int      burnout_debounce;
    float    apogee_vel_mps;      /* baro-derived vz below this ... */
    float    apogee_alt_drop_m;   /* ... AND this far below peak -> FALLING */
    float    recovery_vel_mps;    /* FALLING->RECOVERY when vz above this */
    int      recovery_debounce;
    float    baro_vel_tau_s;      /* vz low-pass time constant; the per-sample
                                   * coefficient is derived from the measured
                                   * baro interval, so the filter keeps its
                                   * response when the baro cadence changes */
    float    baro_vel_min_span_s; /* shortest altitude difference the vz
                                   * estimate is taken over; see flight_sm.c */
    uint32_t post_falling_timeout_ticks;  /* mission_complete latches this long after FALLING entry */
} flight_sm_config_t;

flight_sm_config_t flight_sm_config_default(void);

typedef struct
{
    float az_mps2;      /* body-Z accel, ring-buffer averaged (acc_original[2]) */
    float alt_m;        /* fused altitude (estimator pos[2]) */
    float baro_alt_m;   /* raw barometric altitude */
    bool  baro_fresh;   /* new baro sample this tick */
    float baro_dt_s;    /* measured baro publish interval, s; valid when baro_fresh */
} flight_sm_input_t;

typedef struct
{
    flight_sm_config_t cfg;
    flight_state_t     state;
    int      launch_count, burnout_count, recovery_count;
    float    peak_alt;
    float    prev_baro_alt;
    float    baro_span_s;         /* elapsed time since prev_baro_alt was taken */
    float    baro_vel_z;
    bool     baro_vel_seeded;
    uint32_t tick;
    uint32_t falling_entry_tick;
    bool     mission_complete;
} flight_sm_t;

void        flight_sm_init(flight_sm_t *sm, const flight_sm_config_t *cfg);
bool        flight_sm_step(flight_sm_t *sm, const flight_sm_input_t *in);
const char *flight_state_name(flight_state_t s);
