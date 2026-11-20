/* Airframe-dependent defaults: the parameters to update for a new airframe (current values: Joust 1, K353). */
#pragma once

/* Derivation of each value: docs/design.md#airframe-parameters */

/* Arming (flight/arming.c) */
#define AIRFRAME_CANARD_LOCK_TRAVEL_S      0.5f        /* canard lock clears after this long in free flight */
#define AIRFRAME_ARM_MIN_ALTITUDE_M        50.0f       /* AGL ground interlock; burnout is at about 148 m */

/* Flight state machine (flight/flight_sm.c) */
#define AIRFRAME_MAX_COAST_S               20.0f       /* placeholder until burnout-to-apogee time is known */
#define AIRFRAME_LAUNCH_CONFIRM_S          1.0f
#define AIRFRAME_LAUNCH_CONFIRM_BARO_M     10.0f
#define AIRFRAME_LAUNCH_CONFIRM_IMU_M      1.5f

/* Roll controller (control/roll_ctrl.c) */
#define AIRFRAME_ROLL_PROFILE_MAX_RATE_RADS   1.8326f   /* 105 deg/s */
#define AIRFRAME_ROLL_PROFILE_MAX_ACCEL_RADS2 3.6652f   /* 210 deg/s^2 */
#define AIRFRAME_ROLL_KP_ANGLE             2.5f        /* 1/s */
#define AIRFRAME_ROLL_KP_RATE              0.10f
#define AIRFRAME_ROLL_KI_RATE              0.10f       /* integral time KP/KI = 1 s; retune together */
#define AIRFRAME_ROLL_RATE_INT_LIMIT_RAD   0.1745f     /* 10 deg; must exceed the worst credible trim */
#define AIRFRAME_QBAR_REF_PA               3634.0f     /* dynamic pressure at burnout */
#define AIRFRAME_QBAR_FLOOR_PA             150.0f      /* about 16 m/s, 1.3 s before apogee */
#define AIRFRAME_MAX_DEFLECTION_RAD        0.2618f     /* 15 deg canard hard limit */
