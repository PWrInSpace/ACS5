/* MS5611 HAL-free math: compensation, CRC-4 PROM check (AN520), barometric altitude. */
#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Datasheet compensation with 2nd-order correction; c = PROM C1..C6. int64_t
 * throughout: the datasheet intermediates reach ~58 bits. */
void ms5611_compensate(uint32_t d1, uint32_t d2, const uint16_t c[6],
                       float *pressure_pa, float *temperature_c);

/* CRC-4 over PROM per AN520. prom is 8 words (addresses 0..7). */
bool ms5611_verify_crc4(const uint16_t prom[8]);

/* False when any of C1..C6 is 0x0000 or 0xFFFF: a dead bus reads like that and
 * an all-zero PROM even passes CRC-4. */
bool ms5611_prom_plausible(const uint16_t prom[8]);

/* Hypsometric altitude in metres above the qnh reference. */
float ms5611_pressure_to_altitude(float pressure_pa, float qnh_pa);

/* Interval [s] between two publish timestamps; 0 when prev_us == 0 or the result
 * lies outside the plausible 5 ms .. 0.5 s publish interval. */
float ms5611_interval_s(uint32_t now_us, uint32_t prev_us);
