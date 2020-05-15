#include "callbacks.h"

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2)
	{
		
		// Activation liaison série 2 pour fin envoi message
		// Uniquement après HAL_UART_Transmit_DMA(...)
		tx_busy = false;
		
	}
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2)
	{
		
		// version DMA
		HAL_UART_Receive_DMA(&huart2, rx_buffer, 1); // On lit un buffer constitué de un seul octet
		USART_DMA_ExecuteCommand(rx_buffer[0]);

		// version IT
//		__HAL_UART_CLEAR_IDLEFLAG(&huart2);
//		HAL_UART_Receive_IT(&huart2, &rx_data, 1);
//		if (rx_data == 0x0A) // \n
//		{
//			//executeCommand((uint8_t*)rx_buffer);
//			rx_index = 0;
//			for (uint8_t i = 0; i < 255; i++)
//				rx_buffer[i] = 0; // Clear the buffer
//		}
//		else
//		{
//			rx_buffer[rx_index] = rx_data;
//			rx_index++;
//		}
		
	}
}
