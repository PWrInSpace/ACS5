/* MAX-M10S driver interface used by the GNSS task: configure, drain, monitor. */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "sensor_drivers/gnss/ubx_parse.h"
#include "utils/attributes.h"


typedef struct __UART_HandleTypeDef UART_HandleTypeDef;

typedef enum
{
    MAX_M10S_OK           = 0,   /* success */
    MAX_M10S_ERR_HAL      = 1,   /* STM32 HAL returned HAL_ERROR or HAL_TIMEOUT */
    MAX_M10S_ERR_NAK      = 2,   /* module replied ACK-NAK to a CFG-VALSET key */
    MAX_M10S_ERR_TIMEOUT  = 3,   /* no ACK after all retries */
} max_m10s_status_t;

/* Driver state. Allocate one statically; all fields are private to the driver,
 * use the functions below. */
typedef struct
{
    /* The UART the module is wired to; given to max_m10s_init(). */
    UART_HandleTypeDef *huart;

    /* Interprets the incoming UBX stream. */
    ubx_parser_t  parser;

    /* Ring read pointer, updated by max_m10s_drain(). */
    uint16_t      read_pos;

    /* Receive errors that aborted the DMA (overruns; noise/framing/parity),
     * counted in the UART error callback. */
    volatile uint32_t uart_overruns;
    volatile uint32_t uart_rx_errors;

    /* Set by the UART error callback; max_m10s_drain() then restarts reception. */
    volatile bool rx_restart_pending;

} max_m10s_driver_t;

/* One complete, checksum-verified UBX frame returned by max_m10s_drain(). */
typedef struct
{
    uint8_t        msg_class;
    uint8_t        msg_id;
    uint16_t       len;       /* payload length in bytes */
    const uint8_t *payload;   /* valid only until the next max_m10s_drain() call */
} max_m10s_frame_t;

/* Receive health counters. */
typedef struct
{
    ubx_stats_t parser;      /* frames received, checksum errors, resyncs, ... */
    uint32_t    uart_overruns;
    uint32_t    uart_rx_errors;
} max_m10s_stats_t;

/* Reset driver state and start circular RX DMA on an initialised UART at its
 * current baud; MAX_M10S_ERR_HAL if the DMA cannot start. */
WARN_UNUSED max_m10s_status_t max_m10s_init(max_m10s_driver_t *drv,
                                            UART_HandleTypeDef *huart);

/* Negotiate the flight baud and send the full CFG-VALSET sequence, blocking until
 * every message is ACKed or one fails (NAK, timeout, HAL error); safe to retry. */
WARN_UNUSED max_m10s_status_t max_m10s_configure(max_m10s_driver_t *drv);

/* Feed received DMA bytes to the UBX parser; true with *frame at each complete frame. */
bool max_m10s_drain(max_m10s_driver_t *drv, max_m10s_frame_t *frame);

/* Call from HAL_UART_ErrorCallback() for the GNSS UART. */
void max_m10s_on_uart_error(max_m10s_driver_t *drv);

/* Copy the receive health counters into *out. */
void max_m10s_get_stats(const max_m10s_driver_t *drv, max_m10s_stats_t *out);
