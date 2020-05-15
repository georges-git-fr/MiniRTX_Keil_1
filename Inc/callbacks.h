#ifndef __CALLBACKS_H
#define __CALLBACKS_H

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "math.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "Main.h"

extern UART_HandleTypeDef huart2;

extern uint8_t tx_busy;
//extern uint8_t rx_index;
//extern uint8_t rx_data;
extern uint8_t rx_buffer[1];
extern void USART_DMA_ExecuteCommand(char code);

#endif
