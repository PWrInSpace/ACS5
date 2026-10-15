#include "baro/ms5611_math.h"

#include <math.h>
#include <string.h>

bool ms5611_verify_crc4(const uint16_t prom[8])
{
    const uint16_t crc_stored = prom[7] & 0x000F;

    uint16_t work[8];
    memcpy(work, prom, sizeof(work));
    work[7] &= 0xFF00;

    uint16_t remainder = 0;
    for (int cnt = 0; cnt < 16; ++cnt)
    {
        if (cnt % 2 == 1)
            remainder ^= work[cnt >> 1] & 0x00FF;
        else
            remainder ^= work[cnt >> 1] >> 8;

        for (int bit = 8; bit > 0; --bit)
        {
            if ((remainder & 0x8000) != 0)
                remainder = (uint16_t)((remainder << 1) ^ 0x3000);
            else
                remainder = (uint16_t)(remainder << 1);
        }
    }

    remainder = (remainder >> 12) & 0x000F;
    return remainder == crc_stored;
}

void ms5611_compensate(uint32_t d1, uint32_t d2, const uint16_t c[6],
                       float *pressure_pa, float *temperature_c)
{
    const int64_t dT = (int64_t)d2 - ((int64_t)c[4] << 8);

    int64_t TEMP = 2000 + ((dT * (int64_t)c[5]) >> 23);
    int64_t OFF  = ((int64_t)c[1] << 16) + (((int64_t)c[3] * dT) >> 7);
    int64_t SENS = ((int64_t)c[0] << 15) + (((int64_t)c[2] * dT) >> 8);

    int64_t T2 = 0, OFF2 = 0, SENS2 = 0;
    if (TEMP < 2000)
    {
        const int64_t td = TEMP - 2000;
        T2    = (dT * dT) >> 31;
        OFF2  = (5 * td * td) >> 1;
        SENS2 = (5 * td * td) >> 2;
        if (TEMP < -1500)
        {
            const int64_t td2 = TEMP + 1500;
            OFF2  += 7 * td2 * td2;
            SENS2 += (11 * td2 * td2) >> 1;
        }
    }

    TEMP -= T2;
    OFF  -= OFF2;
    SENS -= SENS2;

    const int64_t P = (((int64_t)d1 * SENS >> 21) - OFF) >> 15;

    *temperature_c = (float)TEMP * 0.01f;   /* centidegrees -> degC */
    *pressure_pa   = (float)P;               /* 0.01 mbar == 1 Pa */
}

float ms5611_pressure_to_altitude(float pressure_pa, float qnh_pa)
{
    /* h = 44330 * (1 - (P/P0)^0.190284) */
    return 44330.0f * (1.0f - powf(pressure_pa / qnh_pa, 0.190284f));
}

/* Plausible barometer publish interval; anything outside is treated as a glitch
 * or a timestamp wrap and reported as "no interval". */
#define MS5611_MIN_INTERVAL_S 0.005f
#define MS5611_MAX_INTERVAL_S 0.5f

float ms5611_interval_s(uint32_t now_us, uint32_t prev_us)
{
    if (prev_us == 0u) return 0.0f;
    float dt_s = (float)(now_us - prev_us) * 1e-6f;
    if (dt_s <= MS5611_MIN_INTERVAL_S || dt_s >= MS5611_MAX_INTERVAL_S) return 0.0f;
    return dt_s;
}
