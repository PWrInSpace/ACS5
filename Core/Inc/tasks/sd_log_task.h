/* SD-card flight log: the flight task publishes records, the SD task writes them. */
#pragma once

#include <stdint.h>

#include "log/log_record.h"

/* Stamp the drop/error counters, seal and queue r without blocking; a full queue drops it. */
void log_publish(log_record_t *r);

/* Records lost to a full queue, and SD write/sync failures, so far. */
void log_stats(uint32_t *dropped, uint32_t *errors);

/* FreeRTOS task entry (registered via osThreadNew in freertos.c). */
void StartSDTask(void *argument);
