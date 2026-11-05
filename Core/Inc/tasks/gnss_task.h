/*
 * ACS5 - GNSS task public interface.
 *
 * The fix snapshot, its mutex and the idle semaphore are private to
 * gnss_task.c. Other tasks read the fix only through gnss_get_fix().
 */
#pragma once

#include <stdbool.h>

#include "flight/flight_data.h"

typedef struct __UART_HandleTypeDef UART_HandleTypeDef;

/*
 * gnss_init - create the task's RTOS objects (fix mutex, wake-up semaphore) and
 * remember the UART the MAX-M10S is connected to, which must already be
 * initialised with its receive DMA linked.
 */
bool gnss_init(UART_HandleTypeDef *huart);

/* FreeRTOS task entry point. Started after a successful gnss_init(); no argument. */
void gnss_task(void *arg);

/*
 * UART event hooks. The HAL has a single, global definition of each UART
 * callback, so the application's central one (usart.c) forwards the events here.
 * Interrupt-safe; they do nothing for a UART other than the GNSS one.
 */
void gnss_task_on_uart_rx_event(UART_HandleTypeDef *huart);
void gnss_task_on_uart_error(UART_HandleTypeDef *huart);

/*
 * gnss_get_fix - copy the latest published fix into *out.
 *
 * Never blocks. Returns false, leaving *out untouched, if the GNSS task has
 * not started yet or the snapshot is being written at this instant; the caller
 * should keep its previous value and try again on its next cycle.
 */
bool gnss_get_fix(gnss_fix_t *out);

/*
 * gnss_capture_datum - ask the task to latch the NED datum from the next
 * valid 3D fix. Call once from the flight state machine while on the pad.
 * Before the datum is latched, ned_m is all zeros and ned_valid is false.
 */
void gnss_capture_datum(void);
