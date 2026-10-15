/*
 * ACS4 Flight Computer - MS5611 Barometric Pressure Sensor Driver
 *
 * MEAS Switzerland MS5611-01BA03, 24-bit delta-sigma ADC barometer.
 * Communicates over SPI (Mode 0 or 3, max 20 MHz).
 *
 * Features:
 * - Non-blocking state machine (driven by external tick)
 * - Factory PROM calibration read with CRC-4 verification (AN520)
 * - Full 2nd-order temperature compensation per datasheet
 * - Configurable oversampling ratio (256-4096)
 * - Output in SI units: Pa, degC
 * - Barometric altitude helper
 *
 * Typical usage:
 *   ms5611_t baro;
 *   ms5611_config_t cfg = ms5611_config_rocket_default();
 *   ms5611_init(&baro, &hspi2, BARO_CS_GPIO_Port, BARO_CS_Pin, &cfg);
 *
 *   // In FlightTask (~100 Hz tick):
 *   ms5611_update(&baro);
 *   if (ms5611_has_new_data(&baro)) {
 *       ms5611_sample_t s = ms5611_sample(&baro);
 *   }
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "sensor_drivers/spi_dev.h"
#include "utils/attributes.h"
#include "main.h" // STM32 HAL types: GPIO_TypeDef, SPI_HandleTypeDef
#include "baro/ms5611_math.h"

/* ========================================================
 * ms5611_sample_t - compensated barometer output
 * ======================================================== */
typedef struct
{
    float    pressure_pa;   /* compensated pressure, Pa */
    float    temperature_c; /* compensated temperature, degC */
    float    altitude_m;    /* barometric altitude (ISA or QNH), m */
    uint32_t timestamp_us;  /* host us (DWT) at end of conversion */
    float    dt_s;          /* interval since previous published sample, s; 0 = unknown */
} ms5611_sample_t;

/* ==================
 * Configuration
 * =================== */
typedef enum
{
    MS5611_OSR_256  = 0x00, /* 0.54 ms typ, 0.065 mbar RMS */
    MS5611_OSR_512  = 0x02, /* 1.06 ms typ, 0.042 mbar RMS */
    MS5611_OSR_1024 = 0x04, /* 2.08 ms typ, 0.027 mbar RMS */
    MS5611_OSR_2048 = 0x06, /* 4.13 ms typ, 0.018 mbar RMS */
    MS5611_OSR_4096 = 0x08, /* 8.22 ms typ, 0.012 mbar RMS */
} ms5611_osr_t;

typedef struct
{
    ms5611_osr_t osr;    /* oversampling ratio for both P and T */
    float        qnh_pa; /* QNH reference pressure for altitude, Pa */
} ms5611_config_t;

ms5611_config_t ms5611_config_rocket_default(void);

/* =====================
 * MS5611 Driver
 * ===================== */
typedef struct
{
    /* Hardware */
    spi_dev_t dev;

    /* Logic state; holds a driver-internal state-machine value (see ms5611.c) */
    uint8_t state;
    bool    initialized;
    bool    new_data;
    /* Reference pressure for altitude_m. The aggregator retargets this to the
     * pad pressure during STANDBY, so altitude_m is metres above the launch
     * site, not above the standard-pressure datum - the SD log's baro_alt
     * column reads AGL. */
    float   qnh_pa;

    ms5611_osr_t osr;
    uint32_t     conv_time_us;

    uint16_t cal[6];
    uint32_t raw_d1;
    uint32_t raw_d2;
    uint8_t  temp_countdown;    /* pressure-only cycles left before D2 is due */

    uint32_t conv_start_us;
    uint32_t error_count;

    uint32_t prev_publish_us;   /* end-of-conversion time of the previous publish, for dt_s */

    ms5611_sample_t last_sample;
} ms5611_t;

WARN_UNUSED bool ms5611_init(ms5611_t *self, SPI_HandleTypeDef *spi,
                             GPIO_TypeDef *cs_port, uint16_t cs_pin,
                             const ms5611_config_t *cfg);
void            ms5611_update(ms5611_t *self);
bool            ms5611_has_new_data(const ms5611_t *self);
ms5611_sample_t ms5611_sample(ms5611_t *self);
void            ms5611_set_qnh(ms5611_t *self, float qnh_pa);
