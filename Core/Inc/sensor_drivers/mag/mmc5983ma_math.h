/* MMC5983MA sample decoding; HAL-free so host tests can use it. */
#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Unpack the three 18-bit axes from registers 0x00..0x06. */
void mmc5983ma_unpack(const uint8_t buf[7], uint32_t raw[3]);

/* One axis in uT: offset binary at 2^17, 16384 counts per gauss, 1 G = 100 uT. */
float mmc5983ma_raw_to_ut(uint32_t raw);

/* True when any axis sits on a rail (0 or 2^18-1): over range or a dead bus. */
bool mmc5983ma_saturated(const uint32_t raw[3]);

/* True when |B| is within tol (fraction) of the reference magnitude. */
bool mmc5983ma_magnitude_plausible(float mag_ut, float ref_ut, float tol);
