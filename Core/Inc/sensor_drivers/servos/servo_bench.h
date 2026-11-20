/*
 * Bench sequence for the servo test build: one fin at a time through
 * +10 / -10 / 0 deg, then the canard lock open / closed. Pure function of
 * time, host-testable.
 */
#pragma once

#include <stdint.h>

#include "sensor_drivers/servos/canard_lock_pulse.h"

#define SERVO_BENCH_FIN_COUNT      4
#define SERVO_BENCH_DEFLECTION_DEG 10.0f

typedef struct
{
    float    fin_deg[SERVO_BENCH_FIN_COUNT];
    uint16_t lock_us;
    int8_t   active_fin;   /* 0..3, or -1 during the lock segment */
} servo_bench_cmd_t;

/* Length of one full cycle; servo_bench_cmd_at repeats with this period. */
uint32_t servo_bench_period_ms(void);

servo_bench_cmd_t servo_bench_cmd_at(uint32_t t_ms);

/* J4 socket driving fin_idx ("J4 CH2".."J4 CH5"), "?" when out of range. */
const char *servo_bench_socket_name(uint8_t fin_idx);
