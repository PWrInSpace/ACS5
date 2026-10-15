#pragma once

#include <stdbool.h>

#include "control/quat.h"

/* Mahony AHRS attitude backend. Swap for a Kalman (or other) backend later
 * behind the same filter/attitude.h facade. */

typedef struct
{
    float integral[3];
    float ref_mag_h[2];
    bool  mag_ref_inited;
} mahony_t;

void mahony_init(mahony_t *m);
void mahony_reset_integral(mahony_t *m);

/* Accel (+ optional mag) feedback correction, then integrate body rates into q.
 * avg[] and mag_ut[] are both body frame; the drivers remap them. */
void mahony_update(mahony_t *m,
                   quat_t *q,
                   quat_t *last_good_q,
                   bool q_valid,
                   const float avg[3],
                   float gx, float gy, float gz,
                   const float mag_ut[3],
                   bool mag_valid,
                   bool calib_done,
                   /* Powered flight and coast: the accelerometer measures drag,
                    * not gravity. Suppresses both feedback corrections and
                    * leaves the quaternion to the gyro. */
                   bool corrections_inhibited,
                   float dt);
