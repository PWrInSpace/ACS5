/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   GPIO for ACS5 / ORC Meteor
  ******************************************************************************
  */
/* USER CODE END Header */

#include "gpio.h"

void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* LEDs idle off (active high through MOSFET/resistor network on Meteor) */
  HAL_GPIO_WritePin(GPIOA, LD1_Pin | LD2_Pin, GPIO_PIN_RESET);

  /* SPI CS idle high */
  HAL_GPIO_WritePin(GPIOD, MAG_CS_Pin | BARO_CS_Pin | IMU_CS_Pin, GPIO_PIN_SET);

  /* Cameras off until launch (only PC1 armed; PC0/PC2 commented out for now) */
#if 0
  HAL_GPIO_WritePin(GPIOC, CAM1_Pin | CAM3_Pin, GPIO_PIN_RESET);
#endif
  HAL_GPIO_WritePin(CAM2_GPIO_Port, CAM2_Pin, GPIO_PIN_RESET);

  /* GPS reset released (active-low style: hold high) */
  HAL_GPIO_WritePin(GPS_RST_GPIO_Port, GPS_RST_Pin, GPIO_PIN_SET);

  /* SD detect */
  GPIO_InitStruct.Pin  = SD_DETECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SD_DETECT_GPIO_Port, &GPIO_InitStruct);

  /* Status LEDs */
  GPIO_InitStruct.Pin   = LD1_Pin | LD2_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Sensor chip-selects */
  GPIO_InitStruct.Pin   = MAG_CS_Pin | BARO_CS_Pin | IMU_CS_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* SPLIT V3 camera MOSFET gate — PC1 only for now */
  GPIO_InitStruct.Pin   = CAM2_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CAM2_GPIO_Port, &GPIO_InitStruct);
#if 0
  GPIO_InitStruct.Pin = CAM1_Pin | CAM3_Pin; /* PC0, PC2 */
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
#endif

  /* GPS RST (out) / INT (in) */
  GPIO_InitStruct.Pin   = GPS_RST_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPS_RST_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin  = GPS_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPS_INT_GPIO_Port, &GPIO_InitStruct);

  /* Sensor INT lines (unused by polled drivers; keep as inputs) */
  GPIO_InitStruct.Pin  = IMU_INT1_Pin | IMU_INT2_Pin | MAG_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

#if 0
  /* CAM3 is on PC2_C (LQFP100 dual-pad): close the analog switch so the
   * digital output reaches the package pin. */
  HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PC2, SYSCFG_SWITCH_PC2_CLOSE);
#endif
}
