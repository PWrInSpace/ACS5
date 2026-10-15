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
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern UART_HandleTypeDef huart7;
extern DMA_HandleTypeDef  hdma_uart7_rx;

void MX_DMA_Init(void);
void MX_UART7_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */
