/*
 * ACS4 Flight Computer - IIM-42653 IMU Driver
 *
 * TDK InvenSense IIM-42653 6-axis IMU (3-axis accel + 3-axis gyro).
 * Communicates over SPI (Mode 0 or 3, max 24 MHz).
 *
 * Features:
 *   - Configurable ODR (up to 32 kHz) and FSR
 *   - Burst read of accel + gyro + temperature in 14 bytes
 *   - DRDY interrupt on INT1 (push-pull, active-high)
 *   - Built-in self-test with factory reference comparison
 *   - Multi-bank register access
 *   - Output in SI units: m/s^2, rad/s, degC
 *
 * Hardware mapping (ACS5 / ORC Meteor):
 *   SPI2: PB13 (SCK) / PB14 / PB15 — IOSwap enabled (Meteor SDI/SDO nets)
 *   CS:   PD13 (IMU_CS)
 *   INT1: PD11 (IMU_INT1)
 *   INT2: PD12 (IMU_INT2)
 *
 * Typical usage:
 *   iim42653_t imu;
 *   iim42653_init(&imu, &hspi2, IMU_CS_GPIO_Port, IMU_CS_Pin);
 *   iim42653_configure(&imu, ... iim42653_config_rocket_default() ...);
 *   iim42653_sample_t sample;
 *   iim42653_read(&imu, &sample);
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "sensor_drivers/spi_dev.h"
#include "utils/attributes.h"
#include "utils/axes_remap.h"
#include "main.h" // STM32 HAL types: GPIO_TypeDef, SPI_HandleTypeDef
#include "spi.h"  // SPI_HandleTypeDef

/* ====================================================
 * iim42653_sample_t - output data posted to the mailbox
 * ==================================================== */

typedef struct
{
    float    accel_mps2[3];       /* X, Y, Z - body frame, m/s^2 */
    float    gyro_rads[3];        /* X, Y, Z - body frame, rad/s */
    float    accel_raw_mps2[3];   /* as the chip reported it, before cfg.axes */
    float    gyro_raw_rads[3];
    float    temp_degc;       /* die temperature, degC (NaN if sensor data not ready) */
    uint32_t timestamp_us;    /* host us, from DWT at the time of the burst read */
} iim42653_sample_t;

/* ======================
 * IIM-42653 Register Map
 * ====================== */

/* BANK 0 (default) */
enum
{
    IIM42653_REG_DEVICE_CONFIG      = 0x11,
    IIM42653_REG_DRIVE_CONFIG       = 0x13,
    IIM42653_REG_INT_CONFIG         = 0x14,
    IIM42653_REG_FIFO_CONFIG        = 0x16,
    IIM42653_REG_TEMP_DATA1         = 0x1D,
    IIM42653_REG_TEMP_DATA0         = 0x1E,
    IIM42653_REG_ACCEL_DATA_X1      = 0x1F,
    IIM42653_REG_ACCEL_DATA_X0      = 0x20,
    IIM42653_REG_ACCEL_DATA_Y1      = 0x21,
    IIM42653_REG_ACCEL_DATA_Y0      = 0x22,
    IIM42653_REG_ACCEL_DATA_Z1      = 0x23,
    IIM42653_REG_ACCEL_DATA_Z0      = 0x24,
    IIM42653_REG_GYRO_DATA_X1       = 0x25,
    IIM42653_REG_GYRO_DATA_X0       = 0x26,
    IIM42653_REG_GYRO_DATA_Y1       = 0x27,
    IIM42653_REG_GYRO_DATA_Y0       = 0x28,
    IIM42653_REG_GYRO_DATA_Z1       = 0x29,
    IIM42653_REG_GYRO_DATA_Z0       = 0x2A,
    IIM42653_REG_INT_STATUS         = 0x2D,
    IIM42653_REG_INTF_CONFIG0       = 0x4C,
    IIM42653_REG_INTF_CONFIG1       = 0x4D,
    IIM42653_REG_PWR_MGMT0          = 0x4E,
    IIM42653_REG_GYRO_CONFIG0       = 0x4F,
    IIM42653_REG_ACCEL_CONFIG0      = 0x50,
    IIM42653_REG_GYRO_CONFIG1       = 0x51,
    IIM42653_REG_GYRO_ACCEL_CONFIG0 = 0x52,
    IIM42653_REG_ACCEL_CONFIG1      = 0x53,
    IIM42653_REG_TMST_CONFIG        = 0x54,
    IIM42653_REG_INT_CONFIG0        = 0x63,
    IIM42653_REG_INT_CONFIG1        = 0x64,
    IIM42653_REG_INT_SOURCE0        = 0x65,
    IIM42653_REG_INT_SOURCE1        = 0x66,
    IIM42653_REG_INT_SOURCE3        = 0x68,
    IIM42653_REG_WHO_AM_I           = 0x75,
    IIM42653_REG_REG_BANK_SEL       = 0x76,
};

/* BANK 1 */
enum
{
    IIM42653_REG_SENSOR_CONFIG0       = 0x03,
    IIM42653_REG_GYRO_CONFIG_STATIC2  = 0x0B,
    IIM42653_REG_GYRO_CONFIG_STATIC3  = 0x0C,
    IIM42653_REG_GYRO_CONFIG_STATIC4  = 0x0D,
    IIM42653_REG_GYRO_CONFIG_STATIC5  = 0x0E,
    IIM42653_REG_GYRO_CONFIG_STATIC6  = 0x0F, /* X NF_COSWZ[7:0] */
    IIM42653_REG_GYRO_CONFIG_STATIC7  = 0x10, /* Y NF_COSWZ[7:0] */
    IIM42653_REG_GYRO_CONFIG_STATIC8  = 0x11, /* Z NF_COSWZ[7:0] */
    IIM42653_REG_GYRO_CONFIG_STATIC9  = 0x12, /* NF_COSWZ[8] + SEL */
    IIM42653_REG_GYRO_CONFIG_STATIC10 = 0x13, /* NF_BW_SEL [6:4] */
    IIM42653_REG_INTF_CONFIG4         = 0x7A,
    IIM42653_REG_INTF_CONFIG5         = 0x7B,
    IIM42653_REG_INTF_CONFIG6         = 0x7C,
};

/* BANK 2 */
enum
{
    IIM42653_REG_ACCEL_CONFIG_STATIC2 = 0x03,
    IIM42653_REG_ACCEL_CONFIG_STATIC3 = 0x04,
    IIM42653_REG_ACCEL_CONFIG_STATIC4 = 0x05,
};


/* Register bit values / masks - Bank 0 */
enum
{
    IIM42653_REG_SOFT_RESET_EN            = 0x01, /* DEVICE_CONFIG */
    IIM42653_REG_SPI_SLEW_DEFAULT         = 0x05, /* DRIVE_CONFIG */
    IIM42653_REG_INT_ACTIVE_HIGH_PP_PULSE = 0x03, /* INT_CONFIG */
    IIM42653_REG_FIFO_BYPASS              = 0x00, /* FIFO_CONFIG */
    IIM42653_REG_DATA_RDY_INT             = 0x08, /* INT_STATUS */
    IIM42653_REG_INTF0_BIGENDIAN_SPI_ONLY = 0x33, /* INTF_CONFIG0 */
    IIM42653_REG_CLKSEL_MASK              = 0x03, /* INTF_CONFIG1 */
    IIM42653_REG_CLKSEL_PLL_AUTO          = 0x01, /* INTF_CONFIG1 */
    IIM42653_REG_PWR_OFF                  = 0x00, /* PWR_MGMT0 */
    IIM42653_REG_PWR_GYRO_ACCEL_LN        = 0x0F, /* PWR_MGMT0 */
    IIM42653_REG_PWR_SENSOR_MASK          = 0x0F, /* PWR_MGMT0 */
    IIM42653_REG_GYRO_CFG1_FILTER_MASK    = 0xEC, /* GYRO_CONFIG1 */
    IIM42653_REG_ACCEL_CFG1_FILT_ORD_MASK = 0x18, /* ACCEL_CONFIG1 */
    IIM42653_REG_TMST_EN_1US              = 0x21, /* TMST_CONFIG */
    IIM42653_REG_INT_CFG0_DRDY_MODE       = 0x20, /* INT_CONFIG0 */
    IIM42653_REG_INT_ASYNC_RESET_BIT      = 1U << 4, /* INT_CONFIG1 */
    IIM42653_REG_UI_DRDY_INT1_EN          = 0x08, /* INT_SOURCE0 */
    IIM42653_REG_WHO_AM_I_VALUE           = 0x44, /* WHO_AM_I */
};

/* Register bit values / masks - Bank 1 */
enum
{
    IIM42653_REG_GYRO_AAF_DIS   = 0x02, /* GYRO_CONFIG_STATIC2 */
    IIM42653_REG_GYRO_NF_DIS    = 0x01, /* GYRO_CONFIG_STATIC2 */
    IIM42653_REG_NF_BW_SEL_MASK = 0x70, /* GYRO_CONFIG_STATIC10 */
    IIM42653_REG_I3C_EN_MASK    = 0x1F, /* INTF_CONFIG6 */
};

/* Register bit values / masks - Bank 2 */
enum
{
    IIM42653_REG_ACCEL_AAF_DIS = 0x01, /* ACCEL_CONFIG_STATIC2 */
};

enum
{
    IIM42653_BURST_LEN = 14,  /* TEMP(2) + ACCEL(6) + GYRO(6) */
};

/* ============
 * Config enums
 * ============ */

/* Gyroscope full-scale range. Bits [7:5] of GYRO_CONFIG0. */
typedef enum
{
    IIM42653_GYRO_FSR_DPS_4000  = 0 << 5, /* +/-4000  deg/s, sensitivity =    8.2 LSB/deg/s */
    IIM42653_GYRO_FSR_DPS_2000  = 1 << 5, /* +/-2000  deg/s, sensitivity =   16.4 LSB/deg/s */
    IIM42653_GYRO_FSR_DPS_1000  = 2 << 5, /* +/-1000  deg/s, sensitivity =   32.8 LSB/deg/s */
    IIM42653_GYRO_FSR_DPS_500   = 3 << 5, /* +/-500   deg/s, sensitivity =   65.5 LSB/deg/s */
    IIM42653_GYRO_FSR_DPS_250   = 4 << 5, /* +/-250   deg/s, sensitivity =  131.0 LSB/deg/s */
    IIM42653_GYRO_FSR_DPS_125   = 5 << 5, /* +/-125   deg/s, sensitivity =  262.0 LSB/deg/s */
    IIM42653_GYRO_FSR_DPS_62_5  = 6 << 5, /* +/-62.5  deg/s, sensitivity =  524.3 LSB/deg/s */
    IIM42653_GYRO_FSR_DPS_31_25 = 7 << 5, /* +/-31.25 deg/s, sensitivity = 1048.6 LSB/deg/s */
} iim42653_gyro_fsr_t;

/* Accelerometer full-scale range. Bits [7:5] of ACCEL_CONFIG0. */
typedef enum
{
    IIM42653_ACCEL_FSR_G_32 = 0 << 5, /* +/-32 g, sensitivity = 1024 LSB/g */
    IIM42653_ACCEL_FSR_G_16 = 1 << 5, /* +/-16 g, sensitivity = 2048 LSB/g */
    IIM42653_ACCEL_FSR_G_8  = 2 << 5, /* +/-8  g, sensitivity = 4096 LSB/g */
    IIM42653_ACCEL_FSR_G_4  = 3 << 5, /* +/-4  g, sensitivity = 8192 LSB/g */
} iim42653_accel_fsr_t;

/* Gyroscope output data rate. Bits [3:0] of GYRO_CONFIG0. */
typedef enum
{
    IIM42653_GYRO_ODR_HZ_32000 = 0x01,
    IIM42653_GYRO_ODR_HZ_16000 = 0x02,
    IIM42653_GYRO_ODR_HZ_8000  = 0x03,
    IIM42653_GYRO_ODR_HZ_4000  = 0x04,
    IIM42653_GYRO_ODR_HZ_2000  = 0x05,
    IIM42653_GYRO_ODR_HZ_1000  = 0x06, /* default */
    IIM42653_GYRO_ODR_HZ_500   = 0x0F,
    IIM42653_GYRO_ODR_HZ_200   = 0x07,
    IIM42653_GYRO_ODR_HZ_100   = 0x08,
    IIM42653_GYRO_ODR_HZ_50    = 0x09,
    IIM42653_GYRO_ODR_HZ_25    = 0x0A,
    IIM42653_GYRO_ODR_HZ_12_5  = 0x0B,
} iim42653_gyro_odr_t;

/* Accelerometer output data rate. Bits [3:0] of ACCEL_CONFIG0. */
typedef enum
{
    IIM42653_ACCEL_ODR_HZ_32000 = 0x01,
    IIM42653_ACCEL_ODR_HZ_16000 = 0x02,
    IIM42653_ACCEL_ODR_HZ_8000  = 0x03,
    IIM42653_ACCEL_ODR_HZ_4000  = 0x04,
    IIM42653_ACCEL_ODR_HZ_2000  = 0x05,
    IIM42653_ACCEL_ODR_HZ_1000  = 0x06, /* default */
    IIM42653_ACCEL_ODR_HZ_500   = 0x0F,
    IIM42653_ACCEL_ODR_HZ_200   = 0x07,
    IIM42653_ACCEL_ODR_HZ_100   = 0x08,
    IIM42653_ACCEL_ODR_HZ_50    = 0x09,
    IIM42653_ACCEL_ODR_HZ_25    = 0x0A,
    IIM42653_ACCEL_ODR_HZ_12_5  = 0x0B,
} iim42653_accel_odr_t;

/* UI filter order (for both gyro and accel). */
typedef enum
{
    IIM42653_FILTER_ORDER_FIRST  = 0,
    IIM42653_FILTER_ORDER_SECOND = 1,
    IIM42653_FILTER_ORDER_THIRD  = 2,
} iim42653_filter_order_t;

/* UI filter bandwidth selection (fraction of ODR). LN mode. */
typedef enum
{
    IIM42653_FILTER_BW_ODR_DIV_2  = 0,
    IIM42653_FILTER_BW_ODR_DIV_4  = 1,
    IIM42653_FILTER_BW_ODR_DIV_5  = 2,
    IIM42653_FILTER_BW_ODR_DIV_8  = 3,
    IIM42653_FILTER_BW_ODR_DIV_10 = 4,
    IIM42653_FILTER_BW_ODR_DIV_16 = 5,
    IIM42653_FILTER_BW_ODR_DIV_20 = 6,
    IIM42653_FILTER_BW_ODR_DIV_40 = 7,
} iim42653_filter_bw_t;

/* Temperature sensor filter bandwidth. Bits [7:5] of GYRO_CONFIG1. */
typedef enum
{
    IIM42653_TEMP_FILTER_BW_HZ_4000 = 0, /* default - not recommended (noisy) */
    IIM42653_TEMP_FILTER_BW_HZ_170  = 1,
    IIM42653_TEMP_FILTER_BW_HZ_82   = 2,
    IIM42653_TEMP_FILTER_BW_HZ_40   = 3, /* recommended for flight */
    IIM42653_TEMP_FILTER_BW_HZ_20   = 4,
    IIM42653_TEMP_FILTER_BW_HZ_10   = 5,
    IIM42653_TEMP_FILTER_BW_HZ_5    = 6,
} iim42653_temp_filter_bw_t;

/* Gyro notch filter bandwidth. Bits [6:4] of GYRO_CONFIG_STATIC10 (Bank 1, 0x13). */
typedef enum
{
    IIM42653_NOTCH_BW_HZ_1449 = 0,
    IIM42653_NOTCH_BW_HZ_680  = 1,
    IIM42653_NOTCH_BW_HZ_329  = 2,
    IIM42653_NOTCH_BW_HZ_162  = 3,
    IIM42653_NOTCH_BW_HZ_80   = 4,
    IIM42653_NOTCH_BW_HZ_40   = 5,
    IIM42653_NOTCH_BW_HZ_20   = 6,
    IIM42653_NOTCH_BW_HZ_10   = 7,
} iim42653_notch_bw_t;

/*
 * Anti-Alias Filter bandwidth preset: the three register parameters (DELT,
 * DELTSQR, BITSHIFT) the hardware needs for a given 3 dB bandwidth. Pick a row
 * from the AAF table in datasheet section 5.3; the three values are not
 * independent, so they must be copied together.
 */
typedef struct
{
    uint8_t  delt;
    uint16_t deltsqr;
    uint8_t  bitshift;
} iim42653_aaf_bw_t;

/*
 * Gyro notch filter configuration. Suppresses MEMS sense resonance noise
 * (1-3 kHz) per axis. freq_hz outside [1000, 3000] disables it.
 */
typedef struct
{
    float               freq_hz; /* notch frequency in Hz (1000-3000), else disabled */
    iim42653_notch_bw_t bw;      /* notch bandwidth */
} iim42653_gyro_notch_config_t;

/* ====================
 * Driver configuration
 * ==================== */

typedef struct
{
    iim42653_gyro_fsr_t          gyro_fsr;
    iim42653_gyro_odr_t          gyro_odr;
    iim42653_accel_fsr_t         accel_fsr;
    iim42653_accel_odr_t         accel_odr;
    iim42653_filter_order_t      gyro_filter_order;
    iim42653_filter_order_t      accel_filter_order;
    iim42653_filter_bw_t         gyro_filter_bw;
    iim42653_filter_bw_t         accel_filter_bw;
    iim42653_temp_filter_bw_t    temp_filter_bw;
    iim42653_aaf_bw_t            gyro_aaf;
    iim42653_aaf_bw_t            accel_aaf;
    iim42653_gyro_notch_config_t gyro_notch;
    axes_remap_t                 axes;  /* sensor to body; configure() rejects a reflection */
    bool                         enable_drdy_int1;
} iim42653_config_t;

/*
 * Default rocket-flight configuration (register-read mode):
 *   - Gyro:  +/-2000 dps @ 1 kHz, 2nd-order filter BW = ODR/4
 *   - Accel: +/-32 g    @ 1 kHz, 2nd-order filter BW = ODR/4
 *   - AAF:   ~536 Hz on both (rejects motor vibration aliasing)
 *   - Notch: enabled at MEMS resonance (~2.5 kHz), BW = 329 Hz
 *   - Temp filter: 40 Hz (low noise for slow-varying temperature)
 *   - DRDY interrupt on INT1, push-pull, active-high
 */
iim42653_config_t iim42653_config_rocket_default(void);

/* ==================
 * IIM-42653 Driver
 * ================== */

typedef struct
{
    /* Hardware */
    spi_dev_t dev;

    iim42653_config_t config;
    uint8_t           current_bank; /* cached selected register bank */
    bool              initialized;
    bool              configured;   /* read() needs cfg.axes, which only configure() installs */
    uint32_t          error_count;

    /* Scale factors: raw * scale = SI unit */
    float accel_scale; /* LSB -> m/s^2 */
    float gyro_scale;  /* LSB -> rad/s */
} iim42653_t;

/*
 * Initialize the IMU: soft reset, WHO_AM_I check, interface config (SPI 4-wire,
 * big-endian), PLL clock select. false on comm failure or wrong WHO_AM_I.
 */
WARN_UNUSED bool iim42653_init(iim42653_t *self, SPI_HandleTypeDef *spi,
                               GPIO_TypeDef *cs_port, uint16_t cs_pin);

/* Configure ODR, FSR, filters, interrupts. Call after init(), before read();
 * sensors are powered on at the end of this call. */
WARN_UNUSED bool iim42653_configure(iim42653_t *self, const iim42653_config_t *cfg);

/* Burst-read accel + gyro + temperature (14 bytes) and convert to SI units. */
WARN_UNUSED bool iim42653_read(iim42653_t *self, iim42653_sample_t *sample);

