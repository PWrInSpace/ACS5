/*
 * MS5611 platform-independent math: temperature/pressure compensation
 * (datasheet + 2nd order), CRC-4 PROM check (AN520), barometric altitude.
 * No RTOS/hardware dependencies - host-testable.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Full MS5611 compensation with 2nd-order correction. All intermediate math
 * uses int64_t to avoid overflow (datasheet intermediates reach ~58 bits).
 * c holds PROM coefficients C1..C6 (index 0..5). Outputs: Pa and degC. */
void ms5611_compensate(uint32_t d1, uint32_t d2, const uint16_t c[6],
                       float *pressure_pa, float *temperature_c);

/* CRC-4 over PROM per AN520. prom is 8 words (addresses 0..7). */
bool ms5611_verify_crc4(const uint16_t prom[8]);

/* Hypsometric altitude in metres above the qnh reference. */
float ms5611_pressure_to_altitude(float pressure_pa, float qnh_pa);

/* Interval [s] between two publish timestamps, or 0 when it cannot be trusted:
 * prev_us == 0 (no previous sample), or the result falls outside a plausible
 * barometer publish interval. The bounds also contain a missed publish or a
 * timestamp discontinuity: the upper one rejects an implausibly long gap, and
 * the lower one assumes the part is not published faster than ~5 ms (true at
 * the OSR-4096 rocket default). */
float ms5611_interval_s(uint32_t now_us, uint32_t prev_us);
