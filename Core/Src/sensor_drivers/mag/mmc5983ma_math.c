#include "sensor_drivers/mag/mmc5983ma_math.h"

#include <math.h>

#define RAW_MAX     0x3FFFFu
#define RAW_OFFSET  131072.0f
#define UT_PER_LSB  (100.0f / 16384.0f)

void mmc5983ma_unpack(const uint8_t buf[7], uint32_t raw[3])
{
    /* Two whole bytes per axis, plus a 2-bit field in buf[6] packed X,Y,Z from the top. */
    for (int i = 0; i < 3; i++)
        raw[i] = ((uint32_t)buf[2 * i] << 10) |
                 ((uint32_t)buf[2 * i + 1] << 2) |
                 ((uint32_t)(buf[6] >> (6 - 2 * i)) & 0x03u);
}

float mmc5983ma_raw_to_ut(uint32_t raw)
{
    return ((float)raw - RAW_OFFSET) * UT_PER_LSB;
}

bool mmc5983ma_saturated(const uint32_t raw[3])
{
    for (int i = 0; i < 3; i++)
        if (raw[i] == 0u || raw[i] >= RAW_MAX)
            return true;
    return false;
}

bool mmc5983ma_magnitude_plausible(float mag_ut, float ref_ut, float tol)
{
    return fabsf(mag_ut - ref_ut) <= tol * ref_ut;
}
