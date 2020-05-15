#include "callbacks.h"

/*----------------------------------------------------------------------------
 Delay_us_TIM
 *---------------------------------------------------------------------------*/
void Delay_us_TIM(uint32_t uSec)
{
	if (uSec < 2)
		uSec = 2;
	usTIM->ARR = uSec - 1; 						/* sets the value in the auto-reload register */
	usTIM->EGR = 1; 									/* Re-initialises the timer */
	usTIM->SR &= ~1; 									/* Resets the flag */
	usTIM->CR1 |= 1; 									/* Enables the counter */
	while((usTIM->SR&0x0001) != 1);
	usTIM->SR &= ~(0x0001);
}

/*----------------------------------------------------------------------------
 sendchar ITM
 *---------------------------------------------------------------------------*/
void sendCharITM(uint8_t c)
{
	ITM_SendChar(c);
}

/*----------------------------------------------------------------------------
 Delay with NOP
 *---------------------------------------------------------------------------*/
void Delay_NOP(uint32_t n)
{
	while(n--)
		__asm__("NOP");
}

/*----------------------------------------------------------------------------
 Delay in microseconds
 *---------------------------------------------------------------------------*/
void Delay_us(uint32_t n)
{
	uint32_t tick;
	uint32_t delayPeriod;
	tick = osKernelSysTick();
	delayPeriod = osKernelSysTickMicroSec(n);
	do{}while((osKernelSysTick() - tick) < delayPeriod);
}

/*----------------------------------------------------------------------------
 USART_DMA_PutString
 *---------------------------------------------------------------------------*/
void USART_DMA_PutString(uint8_t *_emission)
{
	uint16_t l = strlen((char *)_emission);
	
	uint32_t tx_delay_ms = (uint32_t)((float)l * 1.04f) * 2; 			// 1.04ms / octet (à 9600 bauds)
	//uint32_t tx_delay_ms = (uint32_t)((float)l * 0.087f) * 2;		// 87µs(0.087ms) / octet (à 115200 bauds)

	while(tx_busy){osDelay(1);} // Pour être certain que la transmission DMA soit terminée
	tx_busy = true; // à partir de là, on dit que la tx est busy car on va transmettre des données via DMA
	HAL_UART_Transmit_DMA(&huart2, _emission, l); // tx_busy sera remis à false à la fin de la transmission DMA (voir fonction RxCpltCallback dans callbacks.c)
	
	//HAL_UART_Transmit(&huart2, _emission, l, 300);
	
	osDelay(tx_delay_ms);
	
}

/*----------------------------------------------------------------------------
 USART_DMA_ExecuteCommand
 *---------------------------------------------------------------------------*/
void USART_DMA_ExecuteCommand(char code)
{
	char acCommande[80] = {0};
	sprintf(acCommande, "Commande reçue : <%c>\n", code);
	//USART_PutString((uint8_t*)acCommande); 					// pour USART simple
	USART_DMA_PutString((uint8_t*)acCommande); 			// pour USART + DMA
	//printf("Commande reçue : <%c>\n", code); 				// pour fenêtre de debug ITM
}

/*----------------------------------------------------------------------------
 USART_PutChar
 *---------------------------------------------------------------------------*/
void USART_PutChar(uint8_t ch)
{
  while(!(USART2->SR & USART_SR_TXE));
  USART2->DR = ch;
	osDelay(2);
}

/*----------------------------------------------------------------------------
 USART_PutChar
 *---------------------------------------------------------------------------*/
void USART_PutString(uint8_t * str)
{
	while(*str != 0)
	{
		USART_PutChar(*str);
		str++;
	}
}

//int GetChar (void)  {
//  while (!(USART1_SR & USART1_SR_RXNE));
//  return ((int)(USART1_DR & 0xFF));
//}
