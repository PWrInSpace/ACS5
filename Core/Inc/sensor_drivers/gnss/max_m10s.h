/*
 * ACS5 - MAX-M10S HAL interface.
 *
 * This header is the contract between max_m10s.c and the FreeRTOS GNSS task
 * (gnss_task.c). It owns:
 *
 *   - the UART handle used by the STM32 HAL layer;
 *   - the driver state struct (max_m10s_driver_t), allocated statically in
 *     gnss_task.c so no heap is needed;
 *   - the public access and status codes used to configure, drain, and monitor
 *     the MAX-M10S.
 *
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "gnss/ubx_parse.h"
#include "utils/attributes.h"


typedef struct __UART_HandleTypeDef UART_HandleTypeDef;

/* ---------------------------------------------------------- UART
 *
 * ORC Meteor board port labeled TX6/RX6 on PE8/PE7 — silicon AF is UART7.
 */
#define MAX_M10S_HUART_HANDLE   huart7
extern UART_HandleTypeDef MAX_M10S_HUART_HANDLE;

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
    /* Interprets the incoming UBX stream. */
    ubx_parser_t  parser;

    /* Ring read pointer, updated by max_m10s_drain(). */
    uint16_t      read_pos;

    /* Sticky counts of receive errors that aborted the DMA: overruns, and
     * any other UART error (noise, framing, parity). Written from the UART
     * error callback. */
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
    uint32_t frames_ok;
    uint32_t checksum_errors;
    uint32_t resyncs;
    uint32_t oversized;
    uint32_t uart_overruns;
    uint32_t uart_rx_errors;
} max_m10s_stats_t;

/*
 * max_m10s_init - zero driver state and start circular DMA at the UART's
 * current baud.
 *
 * Returns MAX_M10S_OK, or MAX_M10S_ERR_HAL if the DMA could not be started.
 */
WARN_UNUSED max_m10s_status_t max_m10s_init(max_m10s_driver_t *drv);

/*
 * max_m10s_configure - send the full CFG-VALSET sequence and switch baud rate.
 *
 * Blocks until all six messages are ACKed or a failure is detected.
 *
 * Returns MAX_M10S_OK if all six messages are ACKed. Returns MAX_M10S_ERR_NAK or
 * MAX_M10S_ERR_TIMEOUT on any failure. It is safe to retry on failure.
 *
 */
WARN_UNUSED max_m10s_status_t max_m10s_configure(max_m10s_driver_t *drv);

/*
 * max_m10s_drain - consume DMA bytes and feed them to the UBX parser.
 *
 */
bool max_m10s_drain(max_m10s_driver_t *drv, max_m10s_frame_t *frame);

/*
 * max_m10s_on_uart_error - call from HAL_UART_ErrorCallback() for the GNSS UART.
 *
 * A receive error aborts the HAL's DMA transfer, so reception stops until it is
 * restarted. This only counts the error and flags the restart, so it is safe in
 * interrupt context; the next max_m10s_drain() restarts the DMA, resynchronises
 * the ring pointer and parser. The caller must wake whatever calls drain().
 */
void max_m10s_on_uart_error(max_m10s_driver_t *drv);

/*
 * max_m10s_get_stats - copy the receive health counters into *out.
 */
void max_m10s_get_stats(const max_m10s_driver_t *drv, max_m10s_stats_t *out);
