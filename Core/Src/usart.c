/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   UART7 + DMA RX for MAX-M10S GNSS (ACS5 / ORC Meteor)
  ******************************************************************************
  */
/* USER CODE END Header */
#include "usart.h"

UART_HandleTypeDef huart7;
DMA_HandleTypeDef  hdma_uart7_rx;

void MX_DMA_Init(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();

  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
}

void MX_UART7_Init(void)
{
  huart7.Instance                    = UART7;
  huart7.Init.BaudRate               = 38400; /* MAX-M10S boot baud; driver bumps to 115200 */
  huart7.Init.WordLength             = UART_WORDLENGTH_8B;
  huart7.Init.StopBits               = UART_STOPBITS_1;
  huart7.Init.Parity                 = UART_PARITY_NONE;
  huart7.Init.Mode                   = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling           = UART_OVERSAMPLING_16;
  huart7.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
  huart7.Init.ClockPrescaler         = UART_PRESCALER_DIV1;
  huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart7) != HAL_OK)
    Error_Handler();
  if (HAL_UARTEx_SetTxFifoThreshold(&huart7, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
    Error_Handler();
  if (HAL_UARTEx_SetRxFifoThreshold(&huart7, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
    Error_Handler();
  if (HAL_UARTEx_DisableFifoMode(&huart7) != HAL_OK)
    Error_Handler();
}

void HAL_UART_MspInit(UART_HandleTypeDef *uartHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  if (uartHandle->Instance == UART7)
  {
    PeriphClkInitStruct.PeriphClockSelection      = RCC_PERIPHCLK_UART7;
    PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
      Error_Handler();

    __HAL_RCC_UART7_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    /**UART7 GPIO Configuration
      PE7     ------> UART7_RX
      PE8     ------> UART7_TX
    */
    GPIO_InitStruct.Pin       = GPIO_PIN_7 | GPIO_PIN_8;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_UART7;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    hdma_uart7_rx.Instance                 = DMA1_Stream0;
    hdma_uart7_rx.Init.Request             = DMA_REQUEST_UART7_RX;
    hdma_uart7_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_uart7_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_uart7_rx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_uart7_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart7_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_uart7_rx.Init.Mode                = DMA_CIRCULAR;
    hdma_uart7_rx.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_uart7_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_uart7_rx) != HAL_OK)
      Error_Handler();

    __HAL_LINKDMA(uartHandle, hdmarx, hdma_uart7_rx);

    HAL_NVIC_SetPriority(UART7_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(UART7_IRQn);
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *uartHandle)
{
  if (uartHandle->Instance == UART7)
  {
    __HAL_RCC_UART7_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_7 | GPIO_PIN_8);
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_NVIC_DisableIRQ(UART7_IRQn);
  }
}
