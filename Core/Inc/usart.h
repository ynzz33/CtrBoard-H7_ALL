/* USER CODE BEGIN Header */
/*
 * 文件职责：串口硬件、DMA与printf重定向（usart.h）。
 * 本头文件声明本模块的数据结构、常量和调用接口；实现及实际流程见对应源文件。
 * 输入/来源：CubeMX UART/USART配置。
 * 输出/去向：初始化DR16/HI229/VOFA所用串口与DMA/引脚。
 * 关键边界：IDLE新字节获取在uart_idle，协议解析在drivers；这里不重复解析。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
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

extern UART_HandleTypeDef huart8;

extern UART_HandleTypeDef huart9;

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart10;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_UART7_Init(void);
void MX_UART8_Init(void);
void MX_UART9_Init(void);
void MX_USART1_UART_Init(void);
void MX_USART10_UART_Init(void);

/* USER CODE BEGIN Prototypes */
extern DMA_HandleTypeDef hdma_uart7_tx;
extern DMA_HandleTypeDef hdma_uart7_rx;
extern DMA_HandleTypeDef hdma_uart9_rx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

