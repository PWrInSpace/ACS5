#include "sensor_drivers/servos/servo_bench.h"

#include <string.h>

#define STEP_MS          1000u
#define FIN_SEGMENT_MS   (3u * STEP_MS)
#define LOCK_HALF_MS     3000u
#define FINS_SPAN_MS     (SERVO_BENCH_FIN_COUNT * FIN_SEGMENT_MS)
#define PERIOD_MS        (FINS_SPAN_MS + 2u * LOCK_HALF_MS)

uint32_t servo_bench_period_ms(void)
{
    return PERIOD_MS;
}

servo_bench_cmd_t servo_bench_cmd_at(uint32_t t_ms)
{
    servo_bench_cmd_t c;
    memset(&c, 0, sizeof c);   /* zeroes padding too, so commands compare bytewise */

    const uint32_t t = t_ms % PERIOD_MS;

    if (t < FINS_SPAN_MS)
    {
        const uint32_t fin  = t / FIN_SEGMENT_MS;
        const uint32_t step = (t % FIN_SEGMENT_MS) / STEP_MS;

        c.active_fin   = (int8_t)fin;
        c.lock_us      = CANARD_LOCK_US_LOCKED;
        c.fin_deg[fin] = (step == 0u) ?  SERVO_BENCH_DEFLECTION_DEG
                       : (step == 1u) ? -SERVO_BENCH_DEFLECTION_DEG
                       : 0.0f;
    }
    else
    {
        c.active_fin = -1;
        c.lock_us    = (t - FINS_SPAN_MS < LOCK_HALF_MS) ? CANARD_LOCK_US_UNLOCKED
                                                         : CANARD_LOCK_US_LOCKED;
    }
    return c;
}

const char *servo_bench_socket_name(uint8_t fin_idx)
{
    static const char *const names[SERVO_BENCH_FIN_COUNT] = {
        "J4 CH2", "J4 CH3", "J4 CH4", "J4 CH5",
    };
    return (fin_idx < SERVO_BENCH_FIN_COUNT) ? names[fin_idx] : "?";
}
