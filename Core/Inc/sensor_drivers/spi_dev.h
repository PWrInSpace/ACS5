#pragma once

#include "main.h" // STM32 HAL types: GPIO_TypeDef, SPI_HandleTypeDef
#include "spi.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* The IMU's 14-byte temperature + accel + gyro block, the longest burst any
 * sensor here asks for. Transfer buffers hold this plus the address byte. */
#define SPI_DEV_MAX_BURST 14U

/* One software-CS slave. All three sensors share hspi2 and are driven from the
 * aggregator task alone, so transfers need no lock; adding a second caller
 * means adding one here rather than in each driver.
 *
 * They do not share an SCK ceiling - the MMC5983MA is rated 10 MHz against the
 * IIM-42653's 24 MHz - so the baud divider belongs to the device and is
 * reapplied before every transfer. */
typedef struct
{
    SPI_HandleTypeDef *spi;
    GPIO_TypeDef      *cs_port;
    uint16_t           cs_pin;
    uint32_t           baud;    /* CFG1.MBR field, derived from max_sck_hz */
} spi_dev_t;

/* max_sck_hz is the slave's datasheet SCK limit; the fastest divider that
 * respects it is picked against the SPI kernel clock actually in force, so a
 * change to the clock tree cannot silently overclock a part. Leaves CS
 * deasserted. */
void spi_dev_init(spi_dev_t *dev, SPI_HandleTypeDef *spi,
                  GPIO_TypeDef *cs_port, uint16_t cs_pin, uint32_t max_sck_hz);

/* Register access for parts that take an address byte with bit 7 selecting
 * read: IIM-42653 and MMC5983MA both use this convention. */
bool spi_dev_write_reg(spi_dev_t *dev, uint8_t reg, uint8_t val);
bool spi_dev_read_reg(spi_dev_t *dev, uint8_t reg, uint8_t *val);

/* Fails rather than truncating when len exceeds SPI_DEV_MAX_BURST. */
bool spi_dev_read_regs(spi_dev_t *dev, uint8_t reg, uint8_t *buf, size_t len);

/* Command-oriented parts with no address phase (MS5611): clock out cmd, then
 * clock in len bytes with CS held low across both phases. len == 0 sends the
 * command alone. */
bool spi_dev_cmd_read(spi_dev_t *dev, uint8_t cmd, uint8_t *rx, size_t len);
