#include "sensor_drivers/spi_dev.h"

#include <string.h>

/* A hang guard, not a transfer budget: the longest transaction here is the
 * IMU's 15-byte address + burst, 6.4 us of SCK. */
enum { SPI_DEV_TIMEOUT_MS = 10U };
enum { SPI_DEV_READ_FLAG  = 0x80U };

static uint32_t kernel_clock_hz(const SPI_HandleTypeDef *spi)
{
    uint64_t sel = RCC_PERIPHCLK_SPI123;
    if (spi->Instance == SPI4 || spi->Instance == SPI5) sel = RCC_PERIPHCLK_SPI45;
    else if (spi->Instance == SPI6)                     sel = RCC_PERIPHCLK_SPI6;

    return HAL_RCCEx_GetPeriphCLKFreq(sel);
}

/* MBR n divides the kernel clock by 2^(n+1). Returns the fastest setting that
 * still respects max_sck_hz, falling back to the slowest when the kernel clock
 * cannot be read - running a slave over its rated SCK is the failure that must
 * not happen by default. */
static uint32_t baud_for(uint32_t kernel_hz, uint32_t max_sck_hz)
{
    if (kernel_hz != 0U)
    {
        for (uint32_t n = 0U; n < 7U; n++)
        {
            if ((kernel_hz >> (n + 1U)) <= max_sck_hz)
            {
                return n << SPI_CFG1_MBR_Pos;
            }
        }
    }

    return 7U << SPI_CFG1_MBR_Pos;
}

/* CFG1.MBR may only be written while the peripheral is disabled. Every blocking
 * HAL transfer ends in SPI_CloseTransfer, which clears SPE, so the gap between
 * transactions is the one window in which this is legal. */
static void bus_claim(spi_dev_t *dev)
{
    MODIFY_REG(dev->spi->Instance->CFG1, SPI_CFG1_MBR, dev->baud);
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
}

static void bus_release(spi_dev_t *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

void spi_dev_init(spi_dev_t *dev, SPI_HandleTypeDef *spi,
                  GPIO_TypeDef *cs_port, uint16_t cs_pin, uint32_t max_sck_hz)
{
    dev->spi     = spi;
    dev->cs_port = cs_port;
    dev->cs_pin  = cs_pin;
    dev->baud    = baud_for(kernel_clock_hz(spi), max_sck_hz);
    bus_release(dev);
}

bool spi_dev_write_reg(spi_dev_t *dev, uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = {(uint8_t)(reg & ~SPI_DEV_READ_FLAG), val};

    bus_claim(dev);
    const HAL_StatusTypeDef st = HAL_SPI_Transmit(dev->spi, tx, 2U, SPI_DEV_TIMEOUT_MS);
    bus_release(dev);

    return st == HAL_OK;
}

bool spi_dev_read_reg(spi_dev_t *dev, uint8_t reg, uint8_t *val)
{
    uint8_t tx[2] = {(uint8_t)(reg | SPI_DEV_READ_FLAG), 0x00U};
    uint8_t rx[2] = {0x00U, 0x00U};

    bus_claim(dev);
    const HAL_StatusTypeDef st =
        HAL_SPI_TransmitReceive(dev->spi, tx, rx, 2U, SPI_DEV_TIMEOUT_MS);
    bus_release(dev);

    if (st != HAL_OK)
    {
        return false;
    }

    *val = rx[1]; /* rx[0] is clocked out during the address phase */
    return true;
}

bool spi_dev_read_regs(spi_dev_t *dev, uint8_t reg, uint8_t *buf, size_t len)
{
    if (len == 0U || len > SPI_DEV_MAX_BURST)
    {
        return false;
    }

    const size_t total = len + 1U;
    uint8_t tx[SPI_DEV_MAX_BURST + 1U];
    uint8_t rx[SPI_DEV_MAX_BURST + 1U];

    memset(tx, 0x00U, total);
    tx[0] = (uint8_t)(reg | SPI_DEV_READ_FLAG);

    bus_claim(dev);
    const HAL_StatusTypeDef st =
        HAL_SPI_TransmitReceive(dev->spi, tx, rx, (uint16_t)total, SPI_DEV_TIMEOUT_MS);
    bus_release(dev);

    if (st != HAL_OK)
    {
        return false;
    }

    memcpy(buf, &rx[1], len);
    return true;
}

bool spi_dev_cmd_read(spi_dev_t *dev, uint8_t cmd, uint8_t *rx, size_t len)
{
    bus_claim(dev);
    HAL_StatusTypeDef st = HAL_SPI_Transmit(dev->spi, &cmd, 1U, SPI_DEV_TIMEOUT_MS);
    if (st == HAL_OK && len > 0U)
    {
        st = HAL_SPI_Receive(dev->spi, rx, (uint16_t)len, SPI_DEV_TIMEOUT_MS);
    }
    bus_release(dev);

    return st == HAL_OK;
}
