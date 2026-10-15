#include "baro/ms5611.h"
#include "utils/timestamp.h" // timestamp_us()
#include "FreeRTOS.h"
#include "task.h"

#include <math.h>

/* Command Definitions */
enum
{
    CMD_RESET      = 0x1E,
    CMD_ADC_READ   = 0x00,
    CMD_PROM_READ  = 0xA0,
    CMD_CONVERT_D1 = 0x40, /* Pressure */
    CMD_CONVERT_D2 = 0x50, /* Temperature */
};

/* State machine (driver-internal; stored as ms5611_t.state) */
typedef enum
{
    STATE_CONVERT_D1, /* send D1 conversion command (entry point only) */
    STATE_WAIT_D1,    /* wait for D1 -> read D1 + send D2 */
    STATE_WAIT_D2,    /* wait for D2 -> read D2 + compute + send D1 */
} ms5611_state_t;

/* Pressure samples between temperature refreshes. At a 10 ms poll this keeps
 * the compensation temperature under ~0.2 s old while publishing pressure on
 * every cycle. */
#define MS5611_TEMP_EVERY_N 16U

/* Datasheet SCLK maximum for the SPI protocol. */
#define MS5611_MAX_SCK_HZ 20000000U

static void report_error(ms5611_t *self)
{
    self->error_count++;
}

static uint32_t conversion_time_us(ms5611_osr_t osr)
{
    /* Datasheet maximum conversion time per OSR (table "Conversion time tc"),
     * not the typical - the part is read the moment this expires. */
    switch (osr)
    {
        case MS5611_OSR_256:  return 600;
        case MS5611_OSR_512:  return 1170;
        case MS5611_OSR_1024: return 2280;
        case MS5611_OSR_2048: return 4540;
        case MS5611_OSR_4096: return 9040;
        default:              return 9040;
    }
}

/* =======================================================
 * SPI Helpers
 * ======================================================= */

static bool send_command(ms5611_t *self, uint8_t cmd)
{
    if (!spi_dev_cmd_read(&self->dev, cmd, NULL, 0))
    {
        report_error(self);
        return false;
    }
    return true;
}

static bool read_adc(ms5611_t *self, uint32_t *result)
{
    uint8_t rx[3] = {0};

    if (!spi_dev_cmd_read(&self->dev, CMD_ADC_READ, rx, sizeof rx))
    {
        report_error(self);
        return false;
    }

    *result = (rx[0] << 16) | (rx[1] << 8) | rx[2];
    return true;
}

static bool read_prom(ms5611_t *self, uint16_t prom[8])
{
    for (uint8_t i = 0; i < 8; i++)
    {
        uint8_t rx[2] = {0};

        if (!spi_dev_cmd_read(&self->dev, CMD_PROM_READ + (i * 2), rx, sizeof rx))
        {
            report_error(self);
            return false;
        }

        prom[i] = (rx[0] << 8) | rx[1];
    }
    return true;
}

/* =======================================================
 * Initialization
 * ======================================================= */

bool ms5611_init(ms5611_t *self, SPI_HandleTypeDef *spi,
                 GPIO_TypeDef *cs_port, uint16_t cs_pin,
                 const ms5611_config_t *cfg)
{
    spi_dev_init(&self->dev, spi, cs_port, cs_pin, MS5611_MAX_SCK_HZ);
    self->osr          = cfg->osr;
    self->qnh_pa       = cfg->qnh_pa;
    self->conv_time_us = conversion_time_us(self->osr);

    // 1. Reset
    if (!send_command(self, CMD_RESET)) return false;

    // Wait ~3ms for reset sequence to complete.
    // Since init is blocking and done before flight, HAL_Delay or vTaskDelay is fine.
    vTaskDelay(pdMS_TO_TICKS(5));

    // 2. Read PROM
    uint16_t prom[8] = {0};
    if (!read_prom(self, prom)) return false;

    // 3. Verify CRC-4
    if (!ms5611_verify_crc4(prom))
    {
        return false;
    }

    // Save C1-C6 (Addresses 1-6)
    for (int i = 0; i < 6; i++)
    {
        self->cal[i] = prom[i + 1];
    }

    self->initialized      = true;
    self->state            = STATE_CONVERT_D1;
    self->raw_d1           = 0;
    self->raw_d2           = 0;
    self->temp_countdown   = 0;   /* first cycle must read D2: nothing to reuse */
    self->prev_publish_us  = 0;   /* first publish reports dt_s = 0 (no previous) */
    return true;
}

/* =======================================================
 * Non-Blocking State Machine
 * ======================================================= */

static void compute_and_publish(ms5611_t *self)
{
    float pressure_pa = 0;
    float temperature_c = 0;

    ms5611_compensate(self->raw_d1, self->raw_d2, self->cal, &pressure_pa, &temperature_c);

    float altitude_m = ms5611_pressure_to_altitude(pressure_pa, self->qnh_pa);

    /* A short or premature ADC read yields nonsense D1/D2, which compensates to
     * a negative pressure and a NaN altitude. Consumers hold the last published
     * sample, so a poisoned one would never wash out. */
    if (!isfinite(pressure_pa) || !isfinite(altitude_m))
    {
        report_error(self);
        return;
    }

    // timestamp_us() is a register read; take it once and reuse it for both the
    // stamp and the interval so the two cannot disagree.
    uint32_t now_us = timestamp_us();

    taskENTER_CRITICAL();
    self->last_sample.pressure_pa   = pressure_pa;
    self->last_sample.temperature_c = temperature_c;
    self->last_sample.altitude_m    = altitude_m;
    self->last_sample.dt_s          = ms5611_interval_s(now_us, self->prev_publish_us);
    self->last_sample.timestamp_us  = now_us;
    self->prev_publish_us           = now_us;
    self->new_data = true;
    taskEXIT_CRITICAL();
}

/* Kick off the next pressure conversion; on SPI failure the entry state retries
 * it on the following call. */
static void start_pressure_conversion(ms5611_t *self)
{
    if (send_command(self, CMD_CONVERT_D1 + (uint8_t)self->osr))
    {
        self->conv_start_us = timestamp_us();
        self->state         = STATE_WAIT_D1;
    }
    else
    {
        self->state = STATE_CONVERT_D1;
    }
}

void ms5611_update(ms5611_t *self)
{
    if (!self->initialized) return;

    switch (self->state)
    {
        case STATE_CONVERT_D1:
            start_pressure_conversion(self);
            break;

        case STATE_WAIT_D1:
            if ((timestamp_us() - self->conv_start_us) >= self->conv_time_us)
            {
                if (!read_adc(self, &self->raw_d1))
                {
                    self->state = STATE_CONVERT_D1; // Retry on fail
                    break;
                }

                /* Die temperature moves in seconds, pressure in milliseconds.
                 * Refreshing D2 every cycle would halve the pressure rate for
                 * a correction that has not measurably changed. */
                if (self->temp_countdown == 0U)
                {
                    if (send_command(self, CMD_CONVERT_D2 + (uint8_t)self->osr))
                    {
                        self->conv_start_us = timestamp_us();
                        self->state = STATE_WAIT_D2;
                    }
                    else
                    {
                        /* Without this the state stays WAIT_D1 with an expired
                         * timer and nothing ever publishes again. temp_countdown
                         * is left at 0 so the refresh is retried next cycle
                         * rather than deferred another MS5611_TEMP_EVERY_N. */
                        start_pressure_conversion(self);
                    }
                    break;
                }

                self->temp_countdown--;
                compute_and_publish(self);
                start_pressure_conversion(self);
            }
            break;

        case STATE_WAIT_D2:
            if ((timestamp_us() - self->conv_start_us) >= self->conv_time_us)
            {
                if (read_adc(self, &self->raw_d2))
                {
                    self->temp_countdown = MS5611_TEMP_EVERY_N - 1U;
                    compute_and_publish(self);
                }
                start_pressure_conversion(self);
            }
            break;
    }
}

bool ms5611_has_new_data(const ms5611_t *self)
{
    return self->new_data;
}

ms5611_sample_t ms5611_sample(ms5611_t *self)
{
    taskENTER_CRITICAL();
    ms5611_sample_t s = self->last_sample;
    self->new_data = false;
    taskEXIT_CRITICAL();
    return s;
}

void ms5611_set_qnh(ms5611_t *self, float qnh_pa)
{
    self->qnh_pa = qnh_pa;
}

ms5611_config_t ms5611_config_rocket_default(void)
{
    return (ms5611_config_t){
        .osr    = MS5611_OSR_4096,
        .qnh_pa = 101325.0f,
    };
}
