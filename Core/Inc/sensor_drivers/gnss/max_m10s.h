/*
 * ACS5 - MAX-M10S HAL interface.
 *
 * This header is the contract between max_m10s.c and the FreeRTOS GNSS task
 * (gnss_task.c). It owns:
 *
 *   - the circular DMA receive buffer size and UART placeholder used by the
 *     STM32 HAL layer;
 *   - the driver state struct (gnss_driver_t), allocated statically in
 *     gnss_task.c so no heap is needed;
 *   - the public access and status codes used to configure, drain, and monitor
 *     the MAX-M10S.
 *
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "gnss/ubx_parse.h"


typedef struct __UART_HandleTypeDef UART_HandleTypeDef;

/* ---------------------------------------------------------- UART
 *
 * ORC Meteor board port labeled TX6/RX6 on PE8/PE7 — silicon AF is UART7.
 */
#define GNSS_HUART_HANDLE   huart7
extern UART_HandleTypeDef GNSS_HUART_HANDLE;

/* DMA RX ring size. */
#define GNSS_DMA_BUF_LEN    512u


#define GNSS_BAUD_DEFAULT   38400u

#define GNSS_BAUD_FLIGHT    115200u

/* Per-message ACK timeout. */
#define GNSS_CFG_TIMEOUT_MS 200u

/* Retry budget per key before failing configure(). */
#define GNSS_CFG_RETRIES    3u

typedef enum
{
    GNSS_OK           = 0,   /* success */
    GNSS_ERR_HAL      = 1,   /* STM32 HAL returned HAL_ERROR or HAL_TIMEOUT */
    GNSS_ERR_NAK      = 2,   /* module replied ACK-NAK to a CFG-VALSET key */
    GNSS_ERR_TIMEOUT  = 3,   /* no ACK within GNSS_CFG_TIMEOUT_MS × retries */
    GNSS_ERR_OVERFLOW = 4,   /* UART peripheral overrun detected during config */
} gnss_status_t;

/* Driver state. */
typedef struct
{
    //interprets the incoming UBX stream
    ubx_parser_t  parser;

    /* [task] Current NED datum; set after a valid fix. */
    ubx_datum_t   datum;

    /* [task] Capture request flag. */
    volatile bool capture_pending;

    /* [internal] Ring read pointer, updated by max_m10s_drain(). */
    uint16_t      read_pos;

    /* [internal] Sticky UART ORE count. */
    uint32_t      uart_overruns;

} gnss_driver_t;

/*
 * max_m10s_init - zero driver state and start circular DMA at default baud.
 */
void max_m10s_init(gnss_driver_t *drv);

/*
 * max_m10s_configure - send the full CFG-VALSET sequence and switch baud rate.
 *
 * Blocks until all six messages are ACKed or a failure is detected.
 *
 * Returns GNSS_OK if all six messages are ACKed. Returns GNSS_ERR_NAK or
 * GNSS_ERR_TIMEOUT on any failure. It is safe to retry on failure.
 *
 */
gnss_status_t max_m10s_configure(gnss_driver_t *drv);

/*
 * max_m10s_drain - consume DMA bytes, feed them to the UBX parser, and
 * report whether a complete frame was found.
 */
bool max_m10s_drain(gnss_driver_t *drv);
