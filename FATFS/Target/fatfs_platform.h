/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : fatfs_platform.h
  ******************************************************************************
*/
/* USER CODE END Header */
#include "stm32h7xx_hal.h"

#define SD_PRESENT               ((uint8_t)0x01)
#define SD_NOT_PRESENT           ((uint8_t)0x00)
/* ORC Meteor: DETECT_SD = PD0 */
#define SD_DETECT_PIN         GPIO_PIN_0
#define SD_DETECT_GPIO_PORT   GPIOD

uint8_t	BSP_PlatformIsDetected(void);
