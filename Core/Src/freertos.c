/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications (ACS5)
  ******************************************************************************
  */
/* USER CODE END Header */

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os2.h"

/* USER CODE BEGIN Includes */
#include "aggregator.h"
#include "usb_print.h"
#include "ff.h"
/* USER CODE END Includes */

/* USER CODE BEGIN Variables */
extern SPI_HandleTypeDef hspi2;
extern void StartSDTask(void *argument);
extern void gnss_task(void *arg);

extern osThreadId_t sdTaskHandle;
extern osThreadId_t aggregatorTaskHandle;
extern osThreadId_t gnssTaskHandle;

extern const osThreadAttr_t sdTask_attributes;
extern const osThreadAttr_t aggregatorTask_attributes;
extern const osThreadAttr_t gnssTask_attributes;
/* USER CODE END Variables */

void MX_FREERTOS_Init(void)
{
    /* USER CODE BEGIN Init */
    usb_print_init();
    /* USER CODE END Init */

    /* USER CODE BEGIN RTOS_THREADS */
    aggregatorTaskHandle = osThreadNew(StartAggregator, NULL, &aggregatorTask_attributes);
    sdTaskHandle         = osThreadNew(StartSDTask,     NULL, &sdTask_attributes);
    gnssTaskHandle       = osThreadNew(gnss_task,       NULL, &gnssTask_attributes);
    /* USER CODE END RTOS_THREADS */
}
