// SEC_TO_TICKS / HZ_TO_TICKS land within one loop period of the requested wall time.
#include "test_assert.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "utils/control_rate.h"

static void test_sec_to_ticks_lands_on_wall_time(void)
{
    const float secs[] = {0.016f, 0.040f, 0.5f, 6.0f, 240.0f};

    for (unsigned i = 0; i < sizeof secs / sizeof *secs; i++) {
        float wall_ms = (float)SEC_TO_TICKS(secs[i]) * (float)CONTROL_TICK_MS;
        CHECK_NEAR(wall_ms, secs[i] * 1000.0f, (float)CONTROL_TICK_MS);
    }
}

static void test_hz_to_ticks_lands_on_wall_time(void)
{
    const float hz[] = {12.5f, 50.0f};

    for (unsigned i = 0; i < sizeof hz / sizeof *hz; i++) {
        float wall_ms = (float)HZ_TO_TICKS(hz[i]) * (float)CONTROL_TICK_MS;
        CHECK_NEAR(wall_ms, 1000.0f / hz[i], (float)CONTROL_TICK_MS);
    }
}

int main(void)
{
    test_sec_to_ticks_lands_on_wall_time();
    test_hz_to_ticks_lands_on_wall_time();
    return test_report("control_rate");
}
