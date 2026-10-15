/*
 * ACS4 Flight Computer - MMC5983MA Magnetometer Driver
 *
 * MEMSIC MMC5983MA, 18-bit tri-axis magnetometer over SPI.
 * Continuous-measurement mode; output in uT (sensor frame).
 *
 * Typical usage:
 *   mmc5983ma_t mag;
 *   mmc5983ma_config_t cfg = mmc5983ma_config_rocket_default();
 *   mmc5983ma_init(&mag, &hspi2, MAG_CS_GPIO_Port, MAG_CS_Pin);
 *   mmc5983ma_configure(&mag, &cfg);
 *
 *   mmc5983ma_sample_t s;
 *   if (mmc5983ma_read(&mag, &s)) { ... }
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "sensor_drivers/spi_dev.h"
#include "utils/attributes.h"
#include "utils/axes_remap.h"
#include "main.h" // STM32 HAL types: GPIO_TypeDef, SPI_HandleTypeDef

typedef struct
{
    float    field_ut[3];      /* X,Y,Z in uT, body frame */
    float    field_raw_ut[3];  /* as the chip reported it, before cfg.axes */
    uint32_t timestamp_us;
} mmc5983ma_sample_t;

/* =========================
 * Configuration
 * ========================= */

typedef enum
{
    MMC5983MA_BW_100 = 0x00,
    MMC5983MA_BW_200 = 0x01,
    MMC5983MA_BW_400 = 0x02,
    MMC5983MA_BW_800 = 0x03,
} mmc5983ma_bandwidth_t;

typedef enum
{
    MMC5983MA_CM_OFF    = 0x00,
    MMC5983MA_CM_1HZ    = 0x01,
    MMC5983MA_CM_10HZ   = 0x02,
    MMC5983MA_CM_20HZ   = 0x03,
    MMC5983MA_CM_50HZ   = 0x04,
    MMC5983MA_CM_100HZ  = 0x05,
    MMC5983MA_CM_200HZ  = 0x06,
    MMC5983MA_CM_1000HZ = 0x07,
} mmc5983ma_cm_freq_t;

typedef struct
{
    mmc5983ma_bandwidth_t bandwidth;
    mmc5983ma_cm_freq_t   cm_freq;
    bool                  auto_set_reset;
    axes_remap_t          axes;   /* sensor to body; configure() rejects a reflection */
} mmc5983ma_config_t;

mmc5983ma_config_t mmc5983ma_config_rocket_default(void);

/* =========================
 * Driver
 * ========================= */

typedef struct
{
    spi_dev_t dev;

    axes_remap_t axes;
    bool     initialized;
    bool     configured;   /* read() needs cfg.axes, which only configure() installs */
    uint32_t error_count;
} mmc5983ma_t;

WARN_UNUSED bool mmc5983ma_init(mmc5983ma_t *self, SPI_HandleTypeDef *spi,
                                GPIO_TypeDef *cs_port, uint16_t cs_pin);
WARN_UNUSED bool mmc5983ma_configure(mmc5983ma_t *self, const mmc5983ma_config_t *cfg);
WARN_UNUSED bool mmc5983ma_read(mmc5983ma_t *self, mmc5983ma_sample_t *sample);
