/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   UART for ACS5 — GNSS on UART7 (PE8 TX / PE7 RX).
  *
  * Schematic labels this port TX6/RX6 (board "UART6"); on the H725 those pads
  * are UART7 AF7. PD14/PD15 are also labeled TX4/RX4 near the GPS sheet but
  * have no UART TX/RX alternate function, so GNSS rides UART7.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart7;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_UART7_Init(void);

/* USER CODE BEGIN Prototypes */
extern DMA_HandleTypeDef hdma_uart7_rx;
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

