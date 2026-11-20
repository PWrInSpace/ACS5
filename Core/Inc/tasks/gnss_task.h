/* GNSS task interface; other tasks read the fix only through gnss_get_fix(). */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "sensor_drivers/gnss/max_m10s.h"
#include "sensor_drivers/gnss/ubx_parse.h"

/* ------------------------------------------------------------------ GNSS */

/* Link to the GNSS module, distinct from whether it has a fix. */
typedef enum
{
    GNSS_LINK_STARTING = 0,   /* the task has not configured the module yet */
    GNSS_LINK_OK,             /* configured and receiving; see fix_valid for the fix */
    GNSS_LINK_FAILED,         /* configuration is failing; the task keeps retrying */
} gnss_link_t;

/* Latest GNSS snapshot: last NAV-PVT, pad-relative NED, link status, driver counters.
 * NED Down is POSITIVE DOWNWARD (a climbing rocket has negative vel/pos Down). */
typedef struct
{
    /* system time */
    uint32_t timestamp_ms;

    /* ------------------------------------------------------ module link */

    gnss_link_t link;

    /* max_m10s_status_t of the last failure while link == GNSS_LINK_FAILED, else 0. */
    uint8_t  link_error;

    /* Failed configuration attempts so far (saturates at 255). */
    uint8_t  config_failures;

    /* ------------------------------------------------- fix validity gate */

    /* pvt.fix_ok and a 3D or GNSS + dead-reckoning fix; time-only is not valid. */
    bool fix_valid;

    /* ------------------------------------------------- receiver solution */

    /* Last decoded UBX-NAV-PVT, unmodified; all zeros until the first arrives. */
    ubx_nav_pvt_t pvt;

    /* ------------------------------------------ local NED from pad datum */

    /* Position relative to the pad datum; meaningful only when ned_valid. */
    float    ned_m[3];

    /* Datum captured and fix_valid. */
    bool     ned_valid;

    /* ----------------------------------------------- driver health counters */
    max_m10s_stats_t stats;   /* frames, checksum errors, resyncs, UART errors */

} gnss_fix_t;

typedef struct __UART_HandleTypeDef UART_HandleTypeDef;

/* Create the task's RTOS objects and bind the MAX-M10S UART (initialised, RX DMA linked). */
bool gnss_init(UART_HandleTypeDef *huart);

/* FreeRTOS task entry point. Started after a successful gnss_init(); no argument. */
void gnss_task(void *arg);

/* UART event hooks forwarded from the HAL callbacks in usart.c; ISR-safe, ignore other UARTs. */
void gnss_task_on_uart_rx_event(UART_HandleTypeDef *huart);
void gnss_task_on_uart_error(UART_HandleTypeDef *huart);

/* Copy the latest fix into *out without blocking; false (out untouched) if not
 * initialised or the snapshot is being written. */
bool gnss_get_fix(gnss_fix_t *out);

/* Request that the NED datum be latched from the next fix with a position. */
void gnss_capture_datum(void);
