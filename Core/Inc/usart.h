/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   UART for ACS5 — on-board MAX-M10S GNSS on UART9 (PD15 TX / PD14 RX).
  *
  * The GPS sheet nets TX4/RX4 (module TXD/RXD) land on PD14/PD15, which are
  * UART9 AF11 on the H725. PE7/PE8 (TX6/RX6) go to the external "UART6" port.
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

extern UART_HandleTypeDef huart9;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_UART9_Init(void);

/* USER CODE BEGIN Prototypes */
extern DMA_HandleTypeDef hdma_uart9_rx;
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

