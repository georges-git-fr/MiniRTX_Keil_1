#ifndef __INITHAL_H
#define __INITHAL_H

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "math.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "Main.h"

extern TIM_HandleTypeDef htim3;

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;

extern void MX_TIM3_Init(void);
extern void SystemClock_Config(void);
extern void MX_GPIO_Init(void);
extern void MX_DMA_Init(void);
extern void MX_USART2_UART_Init(void);
extern void Error_Handler(void);

#endif
