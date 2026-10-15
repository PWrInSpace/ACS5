#include "mag/mmc5983ma.h"
#include "utils/timestamp.h"

#include "FreeRTOS.h"
#include "task.h"

/* =========================
 * Registers
 * ========================= */
enum
{
    REG_XOUT0   = 0x00,
    REG_STATUS  = 0x08,
    REG_CTRL0   = 0x09,
    REG_CTRL1   = 0x0A,
    REG_CTRL2   = 0x0B,
    REG_PRODUCT = 0x2F,
};

/* bits */
enum
{
    MEAS_M_DONE = 0x01,
    DO_SET      = 0x08,
    DO_RESET    = 0x10,
    AUTO_SR_EN  = 0x20,
    SW_RST      = 0x80,
    CMM_EN      = 0x08,
};

/* 18-bit output is unsigned, centered at 2^17; 16384 counts/Gauss, 1 Gauss = 100 uT */
static const float kScale  = 100.0f / 16384.0f;
static const float kOffset = 131072.0f;

/* Datasheet fc(SCK): "values are guaranteed at 10 MHz clock frequency for SPI,
 * based on characterization results, not tested in production". This is the
 * lowest ceiling on the shared bus - the other two parts take 20 and 24 MHz. */
#define MMC5983MA_MAX_SCK_HZ 10000000U

static void report_error(mmc5983ma_t *self)
{
    self->error_count++;
}

/* =========================
 * SPI helpers
 * ========================= */

static bool write_reg(mmc5983ma_t *self, uint8_t reg, uint8_t val)
{
    if (!spi_dev_write_reg(&self->dev, reg, val))
    {
        report_error(self);
        return false;
    }
    return true;
}

static bool read_reg(mmc5983ma_t *self, uint8_t reg, uint8_t *val)
{
    if (!spi_dev_read_reg(&self->dev, reg, val))
    {
        report_error(self);
        return false;
    }
    return true;
}

static bool read_regs(mmc5983ma_t *self, uint8_t reg, uint8_t *buf, uint8_t len)
{
    if (!spi_dev_read_regs(&self->dev, reg, buf, len))
    {
        report_error(self);
        return false;
    }
    return true;
}

/* =========================
 * Init
 * ========================= */

bool mmc5983ma_init(mmc5983ma_t *self, SPI_HandleTypeDef *spi, GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    spi_dev_init(&self->dev, spi, cs_port, cs_pin, MMC5983MA_MAX_SCK_HZ);
    vTaskDelay(pdMS_TO_TICKS(10));

    /* 2. Software Reset */
    write_reg(self, REG_CTRL1, SW_RST);
    vTaskDelay(pdMS_TO_TICKS(20));

    /* 3. Check ID */
    uint8_t id = 0;
    if (!read_reg(self, REG_PRODUCT, &id) || id != 0x30) {
        return false;
    }

    /* 4. MANDATORY DEGAUSS (Set/Reset Pulse)
     * This clears any "stuck" magnetic states causing the 799.99 uT readings. */
    write_reg(self, REG_CTRL0, DO_SET);
    vTaskDelay(pdMS_TO_TICKS(1));
    write_reg(self, REG_CTRL0, DO_RESET);
    vTaskDelay(pdMS_TO_TICKS(1));

    self->initialized = true;
    self->configured  = false;
    return true;
}

/* =========================
 * Configure
 * ========================= */

bool mmc5983ma_configure(mmc5983ma_t *self, const mmc5983ma_config_t *cfg)
{
    if (!self->initialized) return false;
    if (!axes_remap_is_rotation(&cfg->axes)) return false;

    /* CTRL1: bandwidth */
    if (!write_reg(self, REG_CTRL1, (uint8_t)cfg->bandwidth))
        return false;

    /* CTRL0: auto SR */
    uint8_t ctrl0 = 0;
    if (cfg->auto_set_reset) ctrl0 |= AUTO_SR_EN;

    if (!write_reg(self, REG_CTRL0, ctrl0))
        return false;

    /* CTRL2: continuous */
    uint8_t ctrl2 = (uint8_t)cfg->cm_freq;
    if (cfg->cm_freq != MMC5983MA_CM_OFF)
        ctrl2 |= CMM_EN;

    if (!write_reg(self, REG_CTRL2, ctrl2))
        return false;

    self->axes       = cfg->axes;
    self->configured = true;
    return true;
}

/* =========================
 * Data ready
 * ========================= */

static bool mmc5983ma_data_ready(mmc5983ma_t *self)
{
    uint8_t st = 0;
    if (!read_reg(self, REG_STATUS, &st)) return false;

    return (st & MEAS_M_DONE);
}

/* =========================
 * Read
 * ========================= */

bool mmc5983ma_read(mmc5983ma_t *self, mmc5983ma_sample_t *sample)
{
    /* Without cfg.axes installed there is no body frame to report a sample in,
     * and a zeroed remap would answer with a plausible-looking {0,0,0} field. */
    if (!self->configured) return false;
    if (!mmc5983ma_data_ready(self)) return false;

    uint8_t buf[7];

    if (!read_regs(self, REG_XOUT0, buf, 7))
        return false;

    /* Each axis is 18 bits: two whole bytes plus a 2-bit field in buf[6],
     * packed X,Y,Z from the top of that byte down. */
    for (int i = 0; i < 3; i++)
    {
        uint32_t raw =
            ((uint32_t)buf[2 * i]     << 10) |
            ((uint32_t)buf[2 * i + 1] << 2)  |
            ((buf[6] >> (6 - 2 * i)) & 0x03);

        sample->field_raw_ut[i] = ((float)raw - kOffset) * kScale;
    }

    axes_remap_apply(&self->axes, sample->field_raw_ut, sample->field_ut);

    sample->timestamp_us = timestamp_us();

    return true;
}

/* =========================
 * Config factory
 * ========================= */

mmc5983ma_config_t mmc5983ma_config_rocket_default(void)
{
    return (mmc5983ma_config_t){
        .bandwidth      = MMC5983MA_BW_400,
        .cm_freq        = MMC5983MA_CM_100HZ,
        .auto_set_reset = true,
        /* The board, not the driver, knows how the package is mounted. */
        .axes           = AXES_REMAP_UNSET,
    };
}
