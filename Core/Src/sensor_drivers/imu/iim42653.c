/*
 * IIM-42653 IMU Driver
 *
 * SPI transactions go through spi_dev; this file owns only the register map.
 * Timing: vTaskDelay for ms sleeps, DWT busy-wait for us sleeps.
 */

#include "sensor_drivers/imu/iim42653.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "utils/mathx.h"
#include "utils/timestamp.h"

/* FreeRTOS */
#include "FreeRTOS.h"
#include "task.h"

/* -------------------------------------------------------------------------
 * Static prototypes
 * ------------------------------------------------------------------------- */

static void    iim42653_report_error(iim42653_t *self);

static bool    select_bank(iim42653_t *self, uint8_t bank);
static bool    modify_reg(iim42653_t *self, uint8_t reg, uint8_t clear_mask, uint8_t set_mask);
static bool    modify_bank_reg(iim42653_t *self, uint8_t bank, uint8_t reg,
                               uint8_t clear_mask, uint8_t set_mask);

static float   gyro_sensitivity(iim42653_gyro_fsr_t fsr);
static float   accel_sensitivity(iim42653_accel_fsr_t fsr);
static void    compute_scale_factors(iim42653_t *self);

static bool    has_invalid(const int16_t data[3]);
static void    convert_inertial(const iim42653_t *self, const int16_t accel[3],
                                const int16_t gyro[3], iim42653_sample_t *sample);

static bool    configure_interface(iim42653_t *self);
static bool    configure_odr_fsr(iim42653_t *self, const iim42653_config_t *cfg);
static bool    configure_ui_filters(iim42653_t *self, const iim42653_config_t *cfg);
static bool    configure_aaf(iim42653_t *self, const iim42653_config_t *cfg);
static bool    enable_factory_notch(iim42653_t *self);
static bool    configure_interrupts(iim42653_t *self);
static bool    bypass_fifo(iim42653_t *self);

static bool    iim42653_aaf_bw_is_enabled(const iim42653_aaf_bw_t *aaf);

/* -------------------------------------------------------------------------
 * Write-or-fail macros
 *
 * IMU_TRY:       report_error + return false on failure.
 * IMU_TRY_BANK:  same, but selects bank 0 before returning. Use it for every
 *                failure path opened while a non-zero bank is selected; the
 *                rest of the driver assumes bank 0 on entry.
 * Both reference `self`. Undef'd at end of file.
 * ------------------------------------------------------------------------- */

#define IMU_TRY(expr)                    \
    do                                   \
    {                                    \
        if (!(expr))                     \
        {                                \
            iim42653_report_error(self); \
            return false;                \
        }                                \
    }                                    \
    while (0)

#define IMU_TRY_BANK(expr)               \
    do                                   \
    {                                    \
        if (!(expr))                     \
        {                                \
            iim42653_report_error(self); \
            (void)select_bank(self, 0);  \
            return false;                \
        }                                \
    }                                    \
    while (0)

/* IIM-42653 SPI protocol (Mode 3): an address byte with bit 7 clear to write
 * and set to read, which is what spi_dev implements. */
#define IIM42653_MAX_SCK_HZ 24000000U   /* datasheet: "maximum frequency of SCLK" */

static const float TEMP_SCALE  = 1.0f / 132.48f;
static const float TEMP_OFFSET = 25.0f;

static const int16_t INVALID_RAW = -32768;

/* Microsecond busy-wait using the DWT cycle counter.
 * timestamp_init() must have been called before any driver use. */
static void delay_us(uint32_t us)
{
    const uint32_t start = timestamp_cycles();
    const uint32_t ticks = us * CYCLES_PER_US;
    while ((timestamp_cycles() - start) < ticks)
    {
        /* busy wait */
    }
}

static int16_t parse_be16(const uint8_t *p)
{
    return (int16_t)((uint16_t)((uint16_t)p[0] << 8U) | p[1]);
}

/* -------------------------------------------------------------------------
 * AAF / notch presets + predicates
 * ------------------------------------------------------------------------- */

/* 536 Hz: the widest AAF that still rejects rocket vibration at 1 kHz ODR. */
static const iim42653_aaf_bw_t AAF_BW_536 = {12, 144, 8};

static bool iim42653_aaf_bw_is_enabled(const iim42653_aaf_bw_t *aaf)
{
    return aaf->delt != 0;
}

/* -------------------------------------------------------------------------
 * Config factories
 * ------------------------------------------------------------------------- */

iim42653_config_t iim42653_config_rocket_default(void)
{
    iim42653_config_t cfg = {
        .gyro_fsr           = IIM42653_GYRO_FSR_DPS_2000,
        .gyro_odr           = IIM42653_GYRO_ODR_HZ_1000,
        .accel_fsr          = IIM42653_ACCEL_FSR_G_32,
        .accel_odr          = IIM42653_ACCEL_ODR_HZ_1000,
        .gyro_filter_order  = IIM42653_FILTER_ORDER_SECOND,
        .accel_filter_order = IIM42653_FILTER_ORDER_SECOND,
        /* The loop samples these registers at 500 Hz, so anything above 250 Hz
         * folds back. ODR/4 puts the 3 dB corner at 230.7 Hz, right at Nyquist;
         * ODR/8 moves it to 117.5 Hz and cuts noise bandwidth 223.7 -> 122.7 Hz
         * (datasheet 2nd-order filter table at ODR 1 kHz).
         *
         * The corresponding group delay is 2.1 -> 3.2 ms, so this buys the
         * anti-aliasing and the quieter kd_rate term at the price of 1.1 ms of
         * extra phase lag in the roll loop, on top of the 2 ms sample-and-hold.
         * Any damper retune has to budget for it. */
        .gyro_filter_bw     = IIM42653_FILTER_BW_ODR_DIV_8,
        .accel_filter_bw    = IIM42653_FILTER_BW_ODR_DIV_8,
        .temp_filter_bw     = IIM42653_TEMP_FILTER_BW_HZ_40,
        .gyro_aaf           = AAF_BW_536,
        .accel_aaf          = AAF_BW_536,
        /* The board, not the driver, knows how the package is mounted. */
        .axes               = AXES_REMAP_UNSET,
    };
    return cfg;
}

/* -------------------------------------------------------------------------
 * Bank switching - caches the current bank to skip redundant writes
 * ------------------------------------------------------------------------- */

static bool select_bank(iim42653_t *self, uint8_t bank)
{
    if (bank == self->current_bank)
    {
        return true;
    }

    if (!spi_dev_write_reg(&self->dev, IIM42653_REG_REG_BANK_SEL, bank))
    {
        return false;
    }

    self->current_bank = bank;
    return true;
}

static bool modify_reg(iim42653_t *self, uint8_t reg, uint8_t clear_mask, uint8_t set_mask)
{
    uint8_t val;
    if (!spi_dev_read_reg(&self->dev, reg, &val))
    {
        return false;
    }

    return spi_dev_write_reg(&self->dev, reg, (uint8_t)((val & ~clear_mask) | set_mask));
}

static bool modify_bank_reg(iim42653_t *self, uint8_t bank, uint8_t reg,
                            uint8_t clear_mask, uint8_t set_mask)
{
    if (!select_bank(self, bank))
    {
        return false;
    }

    const bool ok = modify_reg(self, reg, clear_mask, set_mask);
    (void)select_bank(self, 0);
    return ok;
}

/* -------------------------------------------------------------------------
 * Error reporting
 * ------------------------------------------------------------------------- */

static void iim42653_report_error(iim42653_t *self)
{
    self->error_count++;
}

/* -------------------------------------------------------------------------
 * Scale factors
 * ------------------------------------------------------------------------- */

static float gyro_sensitivity(iim42653_gyro_fsr_t fsr)
{
    static const float table[] =
        {8.2f, 16.4f, 32.8f, 65.5f, 131.0f, 262.0f, 524.3f, 1048.6f};
    /* The 3-bit field can only index 0..7, which the table covers exactly. */
    return table[(uint8_t)fsr >> 5U];
}

static float accel_sensitivity(iim42653_accel_fsr_t fsr)
{
    static const float table[] = {1024.0f, 2048.0f, 4096.0f, 8192.0f};
    const unsigned     idx      = (unsigned)((uint8_t)fsr >> 5U);
    configASSERT(idx < 4U);
    return (idx < 4U) ? table[idx] : table[0];
}

static void compute_scale_factors(iim42653_t *self)
{
    self->gyro_scale  = DEG2RAD / gyro_sensitivity(self->config.gyro_fsr);
    self->accel_scale = GRAVITY / accel_sensitivity(self->config.accel_fsr);
}

/* -------------------------------------------------------------------------
 * Conversion helpers
 * ------------------------------------------------------------------------- */

static bool has_invalid(const int16_t data[3])
{
    return data[0] == INVALID_RAW || data[1] == INVALID_RAW || data[2] == INVALID_RAW;
}

/* With invalid-code insertion (INTF_CONFIG0 bit 7 = 0) a valid sample spans -32766..+32767. */
static bool at_full_scale(int16_t v)
{
    return v >= 32767 || v <= -32766;
}

static void convert_inertial(const iim42653_t *self, const int16_t accel[3],
                             const int16_t gyro[3], iim42653_sample_t *sample)
{
    for (int i = 0; i < 3; ++i)
    {
        sample->accel_raw_mps2[i] = (float)accel[i] * self->accel_scale;
        sample->gyro_raw_rads[i]  = (float)gyro[i] * self->gyro_scale;
    }

    axes_remap_apply(&self->config.axes, sample->accel_raw_mps2, sample->accel_mps2);
    axes_remap_apply(&self->config.axes, sample->gyro_raw_rads,  sample->gyro_rads);
}

/* -------------------------------------------------------------------------
 * init()
 * ------------------------------------------------------------------------- */

bool iim42653_init(iim42653_t *self, SPI_HandleTypeDef *spi,
                   GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    spi_dev_init(&self->dev, spi, cs_port, cs_pin, IIM42653_MAX_SCK_HZ);
    self->current_bank = 0U;
    self->initialized  = false;
    self->error_count  = 0U;

    printf("IMU: Sending Soft Reset...\r\n");
    /* An MCU-only reset can leave the IMU in another bank, where 0x11 is a
     * factory notch trim instead of DEVICE_CONFIG. REG_BANK_SEL is in every bank. */
    if (!spi_dev_write_reg(&self->dev, IIM42653_REG_REG_BANK_SEL, 0U) ||
        !spi_dev_write_reg(&self->dev, IIM42653_REG_DEVICE_CONFIG, IIM42653_REG_SOFT_RESET_EN))
    {
        printf("IMU: SPI write FAILED during reset!\r\n");
        return false;
    }

    printf("IMU: Waiting 2ms...\r\n");
    vTaskDelay(pdMS_TO_TICKS(2U));

    printf("IMU: Reading WHO_AM_I...\r\n");
    uint8_t who;
    if (!spi_dev_read_reg(&self->dev, IIM42653_REG_WHO_AM_I, &who))
    {
        printf("IMU: SPI read FAILED!\r\n");
        iim42653_report_error(self);
        return false;
    }

    printf("IMU ID Read: 0x%02X | Expected: 0x%02X\r\n", who, IIM42653_REG_WHO_AM_I_VALUE);
    if (who != IIM42653_REG_WHO_AM_I_VALUE)
    {
        printf("IMU: ID Mismatch!\r\n");
        iim42653_report_error(self);
        return false;
    }

    printf("IMU: Configuring Interface...\r\n");
    if (!configure_interface(self))
    {
        printf("IMU: configure_interface FAILED!\r\n");
        return false;
    }

    printf("IMU: Init SUCCESS!\r\n");
    self->initialized = true;
    self->configured  = false;
    return true;
}

/* -------------------------------------------------------------------------
 * configure_interface()
 * ------------------------------------------------------------------------- */

static bool configure_interface(iim42653_t *self)
{
    /* Clear INT_ASYNC_RESET (bit 4) in INT_CONFIG1. */
    IMU_TRY(modify_reg(self, IIM42653_REG_INT_CONFIG1, IIM42653_REG_INT_ASYNC_RESET_BIT, 0x00U));

    /* Big-endian, FIFO byte count, SPI 4-wire, disable I2C/I3C. */
    IMU_TRY(spi_dev_write_reg(&self->dev, IIM42653_REG_INTF_CONFIG0, IIM42653_REG_INTF0_BIGENDIAN_SPI_ONLY));

    /* Clock source: PLL auto-select (CLKSEL = 01). */
    IMU_TRY(modify_reg(self, IIM42653_REG_INTF_CONFIG1, IIM42653_REG_CLKSEL_MASK,
                       IIM42653_REG_CLKSEL_PLL_AUTO));

    /* SPI slew rate. */
    IMU_TRY(spi_dev_write_reg(&self->dev, IIM42653_REG_DRIVE_CONFIG, IIM42653_REG_SPI_SLEW_DEFAULT));

    /* Disable I3C, keep reserved bit 6. */
    IMU_TRY(modify_bank_reg(self, 1U, IIM42653_REG_INTF_CONFIG6, IIM42653_REG_I3C_EN_MASK, 0x00U));

    return true;
}

/* -------------------------------------------------------------------------
 * configure()
 * ------------------------------------------------------------------------- */

bool iim42653_configure(iim42653_t *self, const iim42653_config_t *cfg)
{
    if (!self->initialized)
    {
        return false;
    }

    if (!axes_remap_is_rotation(&cfg->axes))
    {
        return false;
    }

    self->config = *cfg;
    compute_scale_factors(self);

    IMU_TRY(spi_dev_write_reg(&self->dev, IIM42653_REG_PWR_MGMT0, IIM42653_REG_PWR_OFF));
    delay_us(300U);

    if (!configure_odr_fsr(self, cfg))      return false;
    if (!configure_ui_filters(self, cfg))   return false;
    if (!configure_aaf(self, cfg))          return false;
    if (!enable_factory_notch(self))        return false;
    if (!bypass_fifo(self))                 return false;
    if (!configure_interrupts(self))        return false;

    IMU_TRY(spi_dev_write_reg(&self->dev, IIM42653_REG_PWR_MGMT0, IIM42653_REG_PWR_GYRO_ACCEL_LN));
    vTaskDelay(pdMS_TO_TICKS(50U));

    self->configured = true;
    return true;
}

static bool configure_odr_fsr(iim42653_t *self, const iim42653_config_t *cfg)
{
    const uint8_t gyro_config = (uint8_t)((uint8_t)cfg->gyro_fsr | (uint8_t)cfg->gyro_odr);
    IMU_TRY(spi_dev_write_reg(&self->dev, IIM42653_REG_GYRO_CONFIG0, gyro_config));

    const uint8_t accel_config = (uint8_t)((uint8_t)cfg->accel_fsr | (uint8_t)cfg->accel_odr);
    IMU_TRY(spi_dev_write_reg(&self->dev, IIM42653_REG_ACCEL_CONFIG0, accel_config));

    return true;
}

static bool configure_ui_filters(iim42653_t *self, const iim42653_config_t *cfg)
{
    IMU_TRY(modify_reg(self, IIM42653_REG_GYRO_CONFIG1,
                       IIM42653_REG_GYRO_CFG1_FILTER_MASK,
                       (uint8_t)(((uint8_t)cfg->temp_filter_bw << 5U)
                                 | ((uint8_t)cfg->gyro_filter_order << 2U))));

    IMU_TRY(modify_reg(self, IIM42653_REG_ACCEL_CONFIG1,
                       IIM42653_REG_ACCEL_CFG1_FILT_ORD_MASK,
                       (uint8_t)((uint8_t)cfg->accel_filter_order << 3U)));

    const uint8_t filter_bw =
        (uint8_t)(((uint8_t)cfg->accel_filter_bw << 4U) | (uint8_t)cfg->gyro_filter_bw);
    IMU_TRY(spi_dev_write_reg(&self->dev, IIM42653_REG_GYRO_ACCEL_CONFIG0, filter_bw));

    return true;
}

static bool configure_aaf(iim42653_t *self, const iim42653_config_t *cfg)
{
    /* Gyro AAF - Bank 1 */
    IMU_TRY(select_bank(self, 1U));

    if (iim42653_aaf_bw_is_enabled(&cfg->gyro_aaf))
    {
        IMU_TRY_BANK(spi_dev_write_reg(&self->dev, IIM42653_REG_GYRO_CONFIG_STATIC3,
                               (uint8_t)(cfg->gyro_aaf.delt & 0x3FU)));
        IMU_TRY_BANK(spi_dev_write_reg(&self->dev, IIM42653_REG_GYRO_CONFIG_STATIC4,
                               (uint8_t)(cfg->gyro_aaf.deltsqr & 0xFFU)));
        IMU_TRY_BANK(spi_dev_write_reg(&self->dev, IIM42653_REG_GYRO_CONFIG_STATIC5,
                               (uint8_t)((cfg->gyro_aaf.bitshift << 4U)
                                         | ((cfg->gyro_aaf.deltsqr >> 8U) & 0x0FU))));
        IMU_TRY_BANK(modify_reg(self, IIM42653_REG_GYRO_CONFIG_STATIC2,
                                IIM42653_REG_GYRO_AAF_DIS, 0x00U));
    }
    else
    {
        IMU_TRY_BANK(modify_reg(self, IIM42653_REG_GYRO_CONFIG_STATIC2,
                                0x00U, IIM42653_REG_GYRO_AAF_DIS));
    }
    (void)select_bank(self, 0);

    /* Accel AAF - Bank 2 */
    IMU_TRY(select_bank(self, 2U));

    if (iim42653_aaf_bw_is_enabled(&cfg->accel_aaf))
    {
        IMU_TRY_BANK(spi_dev_write_reg(&self->dev, IIM42653_REG_ACCEL_CONFIG_STATIC2,
                               (uint8_t)((cfg->accel_aaf.delt & 0x3FU) << 1U)));
        IMU_TRY_BANK(spi_dev_write_reg(&self->dev, IIM42653_REG_ACCEL_CONFIG_STATIC3,
                               (uint8_t)(cfg->accel_aaf.deltsqr & 0xFFU)));
        IMU_TRY_BANK(spi_dev_write_reg(&self->dev, IIM42653_REG_ACCEL_CONFIG_STATIC4,
                               (uint8_t)((cfg->accel_aaf.bitshift << 4U)
                                         | ((cfg->accel_aaf.deltsqr >> 8U) & 0x0FU))));
    }
    else
    {
        IMU_TRY_BANK(modify_reg(self, IIM42653_REG_ACCEL_CONFIG_STATIC2,
                                0x00U, IIM42653_REG_ACCEL_AAF_DIS));
    }
    (void)select_bank(self, 0);

    return true;
}

/* The notch coefficients are factory-trimmed per axis to each sense resonance;
 * only make sure the filter is on. */
static bool enable_factory_notch(iim42653_t *self)
{
    IMU_TRY(modify_bank_reg(self, 1U, IIM42653_REG_GYRO_CONFIG_STATIC2,
                            IIM42653_REG_GYRO_NF_DIS, 0x00U));
    return true;
}

/* DATA_RDY must clear on the status read, not on the data read: the burst reads
 * the data registers before INT_STATUS. */
static bool configure_interrupts(iim42653_t *self)
{
    IMU_TRY(spi_dev_write_reg(&self->dev, IIM42653_REG_INT_CONFIG, IIM42653_REG_INT_ACTIVE_HIGH_PP_PULSE));
    IMU_TRY(spi_dev_write_reg(&self->dev, IIM42653_REG_INT_CONFIG0, IIM42653_REG_INT_CFG0_DRDY_CLEAR_ON_STATUS));
    IMU_TRY(spi_dev_write_reg(&self->dev, IIM42653_REG_INT_SOURCE0, IIM42653_REG_UI_DRDY_INT1_EN));

    return true;
}

/* The driver reads the sample registers directly; the FIFO stays bypassed so it
 * cannot fill and raise the overflow interrupt. */
static bool bypass_fifo(iim42653_t *self)
{
    IMU_TRY(spi_dev_write_reg(&self->dev, IIM42653_REG_FIFO_CONFIG, IIM42653_REG_FIFO_BYPASS));
    return true;
}

/* -------------------------------------------------------------------------
 * read()
 * ------------------------------------------------------------------------- */

bool iim42653_read(iim42653_t *self, iim42653_sample_t *sample)
{
    /* Without cfg.axes installed there is no body frame to report a sample in,
     * and a zeroed remap would answer with a plausible-looking {0,0,0}. */
    if (!self->configured)
    {
        return false;
    }

    uint8_t buf[IIM42653_BURST_LEN];
    if (!spi_dev_read_regs(&self->dev, IIM42653_REG_TEMP_DATA1, buf, IIM42653_BURST_LEN))
    {
        iim42653_report_error(self);
        return false;
    }

    const int16_t raw_temp     = parse_be16(&buf[0]);
    const int16_t raw_accel[3] = {
        parse_be16(&buf[2]), parse_be16(&buf[4]), parse_be16(&buf[6])
    };
    const int16_t raw_gyro[3] = {
        parse_be16(&buf[8]), parse_be16(&buf[10]), parse_be16(&buf[12])
    };

    if ((buf[16] & IIM42653_REG_DATA_RDY_INT) == 0U)
    {
        return false;   /* no new sample: the IMU has stalled or reset */
    }

    if (has_invalid(raw_accel) || has_invalid(raw_gyro))
    {
        return false;
    }

    convert_inertial(self, raw_accel, raw_gyro, sample);
    sample->accel_clipped = at_full_scale(raw_accel[0]) || at_full_scale(raw_accel[1]) ||
                            at_full_scale(raw_accel[2]);
    for (int i = 0; i < 3; ++i)
        sample->gyro_clipped[i] = at_full_scale(raw_gyro[i]);

    /* Stamped only once the sample is accepted, so the timestamp never
     * advertises held data as fresh. */
    sample->timestamp_us = timestamp_us();

    sample->temp_degc = (raw_temp == INVALID_RAW)
                            ? NAN
                            : (float)raw_temp * TEMP_SCALE + TEMP_OFFSET;

    return true;
}

#undef IMU_TRY
#undef IMU_TRY_BANK
