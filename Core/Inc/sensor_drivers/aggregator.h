#ifndef AGGREGATOR_H
#define AGGREGATOR_H

#include <stdint.h>

#include "sensor_drivers/imu/iim42653.h"
#include "sensor_drivers/baro/ms5611.h"
#include "sensor_drivers/mag/mmc5983ma.h"

typedef struct
{
    iim42653_sample_t  imu_sample;
    ms5611_sample_t    baro_sample;
    mmc5983ma_sample_t mag_sample;
} flight_data_t;

/* FreeRTOS task entry (registered via osThreadNew in freertos.c). */
void StartAggregator(void *argument);

#endif /* AGGREGATOR_H */
