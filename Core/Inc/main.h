/* USER CODE BEGIN Header */
// Copyright (c) 2026 STMicroelectronics. All rights reserved.
// 本文件遵循软件组件根目录 LICENSE 中的许可条款。
// 如果未附带 LICENSE 文件，则本文件按原样提供。
/* USER CODE END Header */

#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

// 外设初始化失败时进入的统一错误处理函数。
void Error_Handler(void);

// CCD 数据就绪信号：PA0，使用 EXTI0 中断。
#define CCD_DR_Pin GPIO_PIN_0
#define CCD_DR_GPIO_Port GPIOA
#define CCD_DR_EXTI_IRQn EXTI0_IRQn

// CCD 片选信号：PA1。
#define CCD_CS_Pin GPIO_PIN_1
#define CCD_CS_GPIO_Port GPIOA

// 右轮方向信号：PB0。MOTOR1 编号沿用 CubeMX 的引脚标签。
#define MOTOR1_PH_Pin GPIO_PIN_0
#define MOTOR1_PH_GPIO_Port GPIOB

// 左轮方向信号：PB1。MOTOR2 编号沿用 CubeMX 的引脚标签。
#define MOTOR2_PH_Pin GPIO_PIN_1
#define MOTOR2_PH_GPIO_Port GPIOB

#ifdef __cplusplus
}
#endif

#endif // MAIN_H
