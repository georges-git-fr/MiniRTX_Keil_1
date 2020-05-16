/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <initHAL.h>
#include "RTE_Components.h"             // Component selection
#include "stm32f4xx.h"                  // Device header
#include "cmsis_os.h"                   // ARM::CMSIS:RTOS:Keil RTX
#include "EventRecorder.h"              // Keil.ARM Compiler::Compiler:Event Recorder
#include "RTE_Device.h"                 // Keil::Device:STM32Cube Framework:Classic
#include "stm32f4xx_hal.h"              // Keil::Device:STM32Cube HAL:Common
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "Board_LED.h"
#include "tm_stm32f4_rng.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// Les différents templates RTX sont ici : "C:\Users\Utilisateur\AppData\Local\Arm\Packs\ARM\CMSIS\5.7.0\CMSIS\RTOS\RTX\UserCodeTemplates"

//---------------- Disposition des LEDs sur la Discovery F4 ------------------
//
//						orange(1)
//	verte(0)			x				rouge(2)				
//						bleu(3)
//

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

void Delay_us_TIM(uint32_t uSec);
void Delay_NOP(uint32_t n);
void Delay_us(uint32_t n);
void USART_DMA_ExecuteCommand(char code);
void USART_DMA_PutString(uint8_t *_emission);
void sendCharITM(uint8_t c);
void USART_PutChar(uint8_t ch);
void USART_PutString(uint8_t * str);

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define RTE_COMPILER_EVENT_RECORDER

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

uint8_t tx_busy = false;
//uint8_t rx_index = 0;
//uint8_t rx_data = 0x00;
uint8_t rx_buffer[1] = {0};
//uint8_t nbMessageRecu = 0;

volatile uint32_t x, y, dt = 0x00000000U;

//------------------------------------- Déclarations de variables pour démo de "Simple_View.scvd" -----------------------------------------------------
typedef struct
{                        // type definition
  uint32_t hi;           // offset 0
  uint32_t low;          // offset 4
} MyType;
extern volatile MyType mysymbol; // avoid compiler 6 warning
MyType volatile mysymbol;        // symbol definition

void Simple_View_Thread(void const *argument);
osThreadDef(Simple_View_Thread, osPriorityNormal, 1, 0);
osThreadId T_Simple_View_Thread;
//-----------------------------------------------------------------------------------------------------------------------------------------------------
   
//------------------------------------- Déclarations de variables pour démo de "Complex_View.scvd" -----------------------------------------------------
// Type 'A' 
typedef struct TypeA {
  uint32_t                Nummer;
  uint32_t                Status;
  uint32_t                Size;
} TypeA;
 
// Type 'G' 
typedef struct TypeG {
  uint32_t                Nummer;
  uint32_t                Cluster;
  uint32_t                ClusterSize;
  uint32_t                Status;
} TypeG;
 
// Type Array 
typedef struct TypeArray {
  void   *type_ref;
  char    id[2];
  uint16_t attr;
} TypeArray;
 
static TypeA  type_a0 = { 0xA01, 0xA02, 0xA03 };
static TypeA  type_a1 = { 0xA11, 0xA12, 0xA13 };
static TypeG  type_g0 = { 0xF01, 0xF02, 0xF03, 0xF04 };
static TypeG  type_g1 = { 0xF11, 0xF12, 0xF13, 0xF14 };
static TypeG  type_g2 = { 0xF21, 0xF22, 0xF23, 0xF24 };
 
static int32_t x0;
 
extern TypeArray type_array[];
TypeArray type_array[] =  {
  { &x0,      "A0", 1 },
  { &type_g0, "G0", 2 },
  { &type_g1, "G1", 2 },
  { &type_a1, "A1", 1 },
  { &type_g2, "G2", 2 },
};

void Complex_View_Thread(void const *argument);
osThreadDef(Complex_View_Thread, osPriorityNormal, 1, 0);
osThreadId T_Complex_View_Thread;
//-----------------------------------------------------------------------------------------------------------------------------------------------------

// ------------------ Symmetry  --------------------
osSemaphoreDef(sem_Symmetry);
osSemaphoreId sem_Symmetry_ID;									
void Task1_Symmetry(void const *argument);
void Task2_Symmetry(void const *argument);
osThreadId T_Task1_Symmetry;
osThreadId T_Task2_Symmetry;
osThreadDef(Task1_Symmetry, osPriorityNormal, 1, 0);
osThreadDef(Task2_Symmetry, osPriorityNormal, 1, 0);

// ------------------ Moi d'abord, toi ensuite --------------------
void moi_d_abord(void const *argument);
osThreadDef(moi_d_abord, osPriorityNormal, 1, 0);
osThreadId T_moi_d_abord;
void toi_ensuite(void const *argument);
osThreadDef(toi_ensuite, osPriorityNormal, 1, 0);
osThreadId T_toi_ensuite;
osSemaphoreDef(sem_moi_d_abord);
osSemaphoreId sem_moi_d_abord_ID;									

// ------------------------ Flashing LEDs -------------------------
void flashing_LED_0(void const *argument);
osThreadId T_flashing_LED_0;
osThreadDef(flashing_LED_0, osPriorityNormal, 1, 0);
void flashing_LED_1(void const *argument);
osThreadId T_flashing_LED_1;
osThreadDef(flashing_LED_1, osPriorityNormal, 1, 0);
void flashing_LED_2(void const *argument);
osThreadId T_flashing_LED_2;
osThreadDef(flashing_LED_2, osPriorityNormal, 1, 0);
void flashing_LED_3(void const *argument);
osThreadId T_flashing_LED_3;
osThreadDef(flashing_LED_3, osPriorityNormal, 1, 0);

// --------------------- Turnstile - Barrier ----------------------
void turnstile_threadBaseCode(void const *argument);
osThreadDef(turnstile_threadBaseCode, osPriorityNormal, 5, 0);
osThreadId T_turnstile_task0;
osThreadId T_turnstile_task1;
osThreadId T_turnstile_task2;
osThreadId T_turnstile_task3;
osSemaphoreId semTurnstile_ID;									
osSemaphoreDef(semTurnstile);
osSemaphoreId semTurnstile2_ID;									
osSemaphoreDef(semTurnstile2);
osSemaphoreId mutexTurnstile_ID;									
osSemaphoreDef(mutexTurnstile);
unsigned int turnstile_count = 0;

// ------------------------- Rendez-vous --------------------------
osSemaphoreId semArrived1_ID;
osSemaphoreDef(semArrived1);
osSemaphoreId semArrived2_ID;
osSemaphoreDef(semArrived2);
void rendez_vous_Thread1(void const *argument);
osThreadDef(rendez_vous_Thread1, osPriorityNormal, 1, 0);
osThreadId T_rendez_vous1;
void rendez_vous_Thread2(void const *argument);
osThreadDef(rendez_vous_Thread2, osPriorityNormal, 1, 0);
osThreadId T_rendez_vous2;

// ---------------------------- Signal -----------------------------
void signal_Thread1(void const *argument);
osThreadDef(signal_Thread1, osPriorityNormal, 1, 0);
osThreadId T_signal1;
void signal_Thread2(void const *argument);
osThreadDef(signal_Thread2, osPriorityNormal, 1, 0);
osThreadId T_signal2;

// -------------------------- Memory Pool --------------------------
typedef struct
{                                 										// Message object structure
  float voltage;               												// AD result of measured voltage
  float current;               												// AD result of measured current
  int counter;               													// A counter value
} measurement1_st;
 
osPoolDef(mpool, 16, measurement1_st);                    			// Define memory pool
osPoolId mpool_ID;

osMessageQDef(msgQ, 16, &measurement1_st);              				// Define message queue
osMessageQId msgQ_ID;
 
void mpool_producer(void const *argument);         		// forward reference
void mpool_consumer(void const *argument);         		// forward reference
osThreadId T_mpool_producer;                       		// ID for thread 1
osThreadId T_mpool_consumer;                          // for thread 2
osThreadDef(mpool_producer, osPriorityNormal, 1, 0);
osThreadDef(mpool_consumer, osPriorityNormal, 1, 0);

// ---------------------------- Mail box ---------------------------
typedef struct
{
  float voltage;               												// AD result of measured voltage
  float current;               												// AD result of measured current
  int counter;               													// A counter value
} measurement2_st;

osMailQDef(mailBox, 16, measurement2_st);
osMailQId mailBox_ID;

void mail_producer(void const *argument);														
void mail_consumer(void const *argument);
osThreadId T_mail_producer;
osThreadId T_mail_consumer;
osThreadDef(mail_producer, osPriorityNormal, 1,0);
osThreadDef(mail_consumer, osPriorityNormal, 1,0);

// -------------------------- Timer virtuel ------------------------
void callbackTimer(void const *param);
osTimerId timer0_ID;	
osTimerId timer1_ID;	
osTimerId timer2_ID;	
osTimerId timer3_ID;	
osTimerDef(timer0_handle, callbackTimer);
osTimerDef(timer1_handle, callbackTimer);
osTimerDef(timer2_handle, callbackTimer);
osTimerDef(timer3_handle, callbackTimer);

// ---------------------------- Message Q --------------------------
osMessageQId Q_LED;
osMessageQDef(Q_LED, 0x16, unsigned char);
osEvent result;
void msgQ_Thread1(void const *argument);																
osThreadDef(msgQ_Thread1, osPriorityNormal, 1, 0);
osThreadId T_msgQ_Thread1;
void msgQ_Thread2(void const *argument);																
osThreadDef(msgQ_Thread2, osPriorityNormal, 1, 0);
osThreadId T_msgQ_Thread2;

// ---------------------------- Semaphore signalling --------------------------
osSemaphoreId sem1_ID;									
osSemaphoreDef(sem1);
void sem_Thread1(void const *argument);
void sem_Thread2(void const *argument);
osThreadDef(sem_Thread1, osPriorityNormal, 1, 0);
osThreadDef(sem_Thread2, osPriorityNormal, 1, 0);
osThreadId T_semLedOn;
osThreadId T_semLedOff;

// ---------------------------------- Multiplex -------------------------------
osSemaphoreId semMultiplex_ID;
osSemaphoreDef(semMultiplex);
osSemaphoreId semSignal_ID;
osSemaphoreDef(semSignal);
osThreadId T_mux1;																			
osThreadId T_mux2;
osThreadId T_mux3;
osThreadId T_mux4;
osThreadId T_mux5;
osThreadId T_mux6;
void multiplex_Thread(void const *argument);
osThreadDef(multiplex_Thread, osPriorityNormal, 5, 0);

// ---------------------------------- Send usart ----------------------------
osThreadId T_send_usart2;
void send_usart2_Task(void const *argument);
osThreadDef(send_usart2_Task, osPriorityNormal, 1, 0);

// --------------------------  Mutex avec sortie ITM -----------------------
osMutexId sendITM_mutex_ID;
osMutexDef(sendITM_mutex);
osThreadId T_sendITM1;
osThreadId T_sendITM2;
void sendITM_Thread1(void const *argument);
void sendITM_Thread2(void const *argument);
osThreadDef(sendITM_Thread1, osPriorityNormal, 1, 0);
osThreadDef(sendITM_Thread2, osPriorityNormal, 1, 0);
	
// -------------------------  Mutex avec sortie usart ----------------------
osMutexId sendUSART_mutex_ID;
osMutexDef(sendUSART_mutex);
osThreadId T_sendUSART1;
osThreadId T_sendUSART2;
void sendUSART_Thread1(void const *argument);
void sendUSART_Thread2(void const *argument);
osThreadDef(sendUSART_Thread1, osPriorityNormal, 1, 0);
osThreadDef(sendUSART_Thread2, osPriorityNormal, 1, 0);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void MX_USART2_UART_Init(void);
void MX_TIM3_Init(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*----------------------------------------------------------------------------
	Pour démo de "Simple_View.scvd"
 *---------------------------------------------------------------------------*/
void Simple_View_Thread(void const *argument) 
{
	uint32_t i = 0;
	for(;;) 
	{	
		for(i=0; i<=30; i++ )
		{
      mysymbol.hi = i*i;
      mysymbol.low = i;
			osDelay(100);
    }
	}
}

/*----------------------------------------------------------------------------
  Pour démo de "Complex_View.scvd"
 *---------------------------------------------------------------------------*/
void Complex_View_Thread(void const *argument) 
{
	type_array[0].type_ref =  &type_a0;
	for (;;) 
	{	
		osDelay(1);
	}
}

/*----------------------------------------------------------------------------
  Task1_Symmetry
 *---------------------------------------------------------------------------*/
void Task1_Symmetry(void const *argument)
{
	#define TEST4
	
	sem_Symmetry_ID = osSemaphoreCreate(osSemaphore(sem_Symmetry), 0);
	
	while (1)
	{
		#ifdef TEST1
		osSemaphoreWait(sem_Symmetry_ID, osWaitForever);
		LED_On(0);
		DWT->CYCCNT = 0;
		#endif
		
		#ifdef TEST2
		osDelay(1);
		LED_Off(0);
		osSemaphoreWait(sem_Symmetry_ID, 1U);
		LED_Off(0);
		DWT->CYCCNT = 0;
		#endif
		
		#ifdef TEST3
		osSemaphoreRelease(sem_Symmetry_ID);
		osDelay(1);
		LED_Off(0);
		DWT->CYCCNT = 0;
		osSemaphoreWait(sem_Symmetry_ID, osWaitForever);
		LED_On(0);
		y = DWT->CYCCNT; // Elapsed clock ticks, at SystemCoreClock
		dt = (y - x) * 6; // en ns, le 6 représente le tick atomique liée à la fréquence de base (1/168000000Hz)=0.000000006s => 6ns
		#endif
		
		#ifdef TEST4
		osDelay(1);
		LED_Off(0);
		DWT->CYCCNT = 0;
		osSemaphoreRelease(sem_Symmetry_ID);
		LED_On(0);
		osSemaphoreWait(sem_Symmetry_ID, osWaitForever);
		y = DWT->CYCCNT; // Elapsed clock ticks, at SystemCoreClock
		dt = (y - x) * 6; // en ns, le 6 représente le tick atomique liée à la fréquence de base (1/168000000Hz)=0.000000006s => 6ns
		#endif

		//osDelay(1);
		
	}
}

/*----------------------------------------------------------------------------
  Task2_Symmetry
 *---------------------------------------------------------------------------*/
void Task2_Symmetry(void const *argument)
{
	while (1)
	{
		#ifdef TEST1
		osDelay(1);
		LED_Off(0);                
		y = DWT->CYCCNT; // Elapsed clock ticks, at SystemCoreClock
		dt = (y - x) * 6; // en ns, le 6 représente le tick atomique liée à la fréquence de base (1/168000000Hz)=0.000000006s => 6ns
		osSemaphoreRelease(sem_Symmetry_ID);
		#endif
		
		#ifdef TEST2
		LED_On(0);
		y = DWT->CYCCNT; // Elapsed clock ticks, at SystemCoreClock
		dt = (y - x) * 6; // en ns, le 6 représente le tick atomique liée à la fréquence de base (1/168000000Hz)=0.000000006s => 6ns
		#endif
		
		#ifdef TEST3
		osDelay(1);
		#endif
		
		#ifdef TEST4
		osDelay(1);
		#endif
		
		//osDelay(1);

	}
}

/*----------------------------------------------------------------------------
  Flashing LED 0 (verte à 9h)
 *---------------------------------------------------------------------------*/
void flashing_LED_0(void const *argument)
{
//	uint32_t r = 500; // par défaut sans random
//	uint32_t min = 100;
//	uint32_t max = 1000;
//	uint32_t random = (uint32_t) argument;
//	if (random == 0x01U)
//		r = (TM_RNG_Get() % (max + 1 - min)) + min;
	for (;;) 
	{
		LED_On(0);
		x = DWT->CYCCNT; // On lit le compteur avant le processus
		//Delay_us(1);
    //Delay_us_TIM(65000);
		osDelay(500);
		y = DWT->CYCCNT; // On lit le compteur après le processus
		dt = (y - x) * 6; // en ns, le 6 représente le tick atomique liée à la fréquence de base (1/168000000Hz)=0.000000006s => 6ns
		LED_Off(0);
		//Delay_us(1);
    //Delay_us_TIM(65000);
	  osDelay(500);
	}
}

/*----------------------------------------------------------------------------
  Flashing LED 1 (orange à 12h)
 *---------------------------------------------------------------------------*/
void flashing_LED_1(void const *argument)
{
//	uint32_t r = 500; // par défaut sans random
//	uint32_t min = 100;
//	uint32_t max = 1000;
//	uint32_t random = (uint32_t) argument;
//	if (random == 0x01U)
//		r = (TM_RNG_Get() % (max + 1 - min)) + min;
	for (;;) 
	{
		LED_On(1);                
		//Delay_us(65000);
    //Delay_us_TIM(65000);
		osDelay(65);
		LED_Off(1);
		//Delay_us(65000);
    //Delay_us_TIM(65000);
		osDelay(65);
	}
}

/*----------------------------------------------------------------------------
  Flashing LED 2 (rouge à 3h)
 *---------------------------------------------------------------------------*/
void flashing_LED_2(void const *argument)
{
	uint32_t r = 500; // par défaut sans random
	uint32_t min = 100;
	uint32_t max = 1000;
	uint32_t random = (uint32_t) argument;
	if (random == 0x01U)
		r = (TM_RNG_Get() % (max + 1 - min)) + min;
	for (;;) 
	{
		LED_On(2);                
		osDelay(r);
		LED_Off(2);
		osDelay(r);
	}
}

/*----------------------------------------------------------------------------
  Flashing LED 3 (bleue à 6h)
 *---------------------------------------------------------------------------*/
void flashing_LED_3(void const *argument)
{
	uint32_t r = 500; // par défaut sans random
	uint32_t min = 100;
	uint32_t max = 1000;
	uint32_t random = (uint32_t) argument;
	if (random == 0x01U)
		r = (TM_RNG_Get() % (max + 1 - min)) + min;
	for (;;) 
	{
		LED_On(3);                
		osDelay(r);
		LED_Off(3);
		osDelay(r);
	}
}

/*----------------------------------------------------------------------------
  Moi d'abord
 *---------------------------------------------------------------------------*/
void moi_d_abord(void const *argument)
{
	uint32_t r = 0;
	uint32_t min = 100;
	uint32_t max = 1000;
	sem_moi_d_abord_ID = osSemaphoreCreate(osSemaphore(sem_moi_d_abord), 0);
	//osSemaphoreRelease(sem_moi_d_abord_ID);
	while(1)
	{
		printf("Moi d'abord\n");
		r = (TM_RNG_Get() % (max + 1 - min)) + min;
		osDelay(r);
		osSemaphoreRelease(sem_moi_d_abord_ID);
	}
}

/*----------------------------------------------------------------------------
  Moi ensuite
 *---------------------------------------------------------------------------*/
void toi_ensuite(void const *argument)
{
	while(1)
	{
		osSemaphoreWait(sem_moi_d_abord_ID, osWaitForever);
		printf("            Toi ensuite\n");
	}
}

/*----------------------------------------------------------------------------
  Synchronisation avec algo "Barrier"
 *---------------------------------------------------------------------------*/
void turnstile_threadBaseCode(void const *argument)
{
	#define TURNSTILE
	uint32_t r = 0;
	uint32_t min = 100;
	uint32_t max = 2500;
	uint32_t LED_data;
	LED_data = (uint32_t) argument;
	while(1)					
	{
		#ifdef TURNSTILE
		//-----------------------------------Entry Turnstile-------------------------------			
		osSemaphoreWait(mutexTurnstile_ID, osWaitForever);			 				// Allow one task at a time to access the first turnstile
		turnstile_count = turnstile_count + 1;					 												// Increment count
		if (turnstile_count == 4) 					 													// When last section of code reaches this point run his code
		{
			osSemaphoreWait(semTurnstile2_ID, osWaitForever);	 			// Lock the second turnstile
			osSemaphoreRelease(semTurnstile_ID);				 						// Unlock the first turnstile
		}			
		osSemaphoreRelease(mutexTurnstile_ID);													// Allow other tasks to access the turnstile
		
		osSemaphoreWait(semTurnstile_ID, osWaitForever);					// Turnstile Gate			
		osSemaphoreRelease(semTurnstile_ID);
		//-----------------------------------Entry Turnstile-----------------------------------
		#endif

		LED_On(LED_data);
		printf("            LED_On(%d)\n", LED_data);
		r = (TM_RNG_Get() % (max + 1 - min)) + min;
		osDelay(r);
		LED_Off(LED_data);
		printf("LED_Off(%d)\n", LED_data);
		//r = (TM_RNG_Get() % (max + 1 - min)) + min;
		//osDelay(((LED_data+1)*r));
		osDelay(r);

		#ifdef TURNSTILE
		//----------------------------------Exit Turnstile------------------------------------
		osSemaphoreWait(mutexTurnstile_ID, osWaitForever);			 				// Allow one task at a time to access the turnstile
		turnstile_count = turnstile_count - 1;
		if (turnstile_count == 0)						 													// When last section of code reaches this point run his code
		{
			osSemaphoreWait(semTurnstile_ID, osWaitForever);		 		// Lock the second turnstile
			osSemaphoreRelease(semTurnstile2_ID);			 							// Unlock the first turnstile
		}
		osSemaphoreRelease(mutexTurnstile_ID);					 								// Allow other tasks to access the turnstile
		
		osSemaphoreWait(semTurnstile2_ID, osWaitForever);		 			// Turnstile Gate
		osSemaphoreRelease(semTurnstile2_ID);
		//---------------------------------Exit Turnstile-------------------------------------
		#endif
	}
}		

/*----------------------------------------------------------------------------
  Synchronise the LED's using a semaphore rendez-vous
 *---------------------------------------------------------------------------*/
void rendez_vous_Thread1(void const *argument)
{
	#define RENDEZ_VOUS
	uint32_t r = 0;
	uint32_t min = 100;
	uint32_t max = 1000;
	for (;;)
	{
		LED_Off(0);
		printf("LED_Off(0)\n");
		r = (TM_RNG_Get() % (max + 1 - min)) + min;
		osDelay(r);
		#ifdef RENDEZ_VOUS
		osSemaphoreRelease(semArrived2_ID);								//The semaphores ensure both tasks arrive here
		osSemaphoreWait(semArrived1_ID, osWaitForever);		//before continuing
		#endif
		
		LED_On(0);
		printf("            LED_On(0) = Synchronisation avec LED 1\n"); // Synchronisation des deux threads ici
		//r = (TM_RNG_Get() % (max + 1 - min)) + min;
		osDelay(r);
		
		#ifdef RENDEZ_VOUS
		osSemaphoreRelease(semArrived2_ID);								//The semaphores ensure both tasks arrive here
		osSemaphoreWait(semArrived1_ID, osWaitForever);		//before continuing
		#endif
 }
}

/*----------------------------------------------------------------------------
   Synchronise the LED's using a semaphore rendez-vous
 *---------------------------------------------------------------------------*/
void rendez_vous_Thread2(void const *argument)
{
	uint32_t r = 0;
	uint32_t min = 100;
	uint32_t max = 1000;
	for (;;) 
	{
		LED_Off(1); 
		printf("LED_Off(1)\n");
		r = (TM_RNG_Get() % (max + 1 - min)) + min;
		osDelay(r);
		#ifdef RENDEZ_VOUS
		osSemaphoreRelease(semArrived1_ID);								//The semaphores ensure both tasks arrive here
		osSemaphoreWait(semArrived2_ID, osWaitForever);		//before continuing
		#endif
		
		LED_On(1);
		printf("            LED_On(1) = Synchronisation avec LED 0\n"); // Synchronisation des deux threads ici
		//r = (TM_RNG_Get() % (max + 1 - min)) + min;
		osDelay(r);
		
		#ifdef RENDEZ_VOUS
		osSemaphoreRelease(semArrived1_ID);								//The semaphores ensure both tasks arrive here
		osSemaphoreWait(semArrived2_ID, osWaitForever);		//before continuing
		#endif
	}
}

/*----------------------------------------------------------------------------
  Flash LED 0 when signaled by the signal_Thread2
 *---------------------------------------------------------------------------*/
void signal_Thread1(void const *argument)
{
	for (;;) 
	{
		osSignalWait(0xAB, osWaitForever);
		LED_On(0);                          
		osSignalWait(0x45, osWaitForever);	
		LED_Off(0);
	}
}
/*----------------------------------------------------------------------------
  Flash LED 1 and synchronise the flashing of LED 0 by setting a signal flag
 *---------------------------------------------------------------------------*/
void signal_Thread2(void const *argument)
{
	for (;;) 
	{
		LED_On(1);		
		osSignalSet(T_signal1, 0xAB);
		osDelay(250);
		LED_Off(1);
		osSignalSet(T_signal1, 0x45);
		osDelay(1000);
	}
}

/*----------------------------------------------------------------------------
	mpool_producer
 *---------------------------------------------------------------------------*/
void mpool_producer(void const *argument)
{
  measurement1_st *mptr;
 
	while (1) 
	{
		mptr = (measurement1_st*)osPoolAlloc(mpool_ID);						//Allocate a memory pool buffer
		mptr->voltage = 1.23456789;
		mptr->current = 0.0987654321;
		mptr->counter = 66006;
		osMessagePut(msgQ_ID, (uint32_t)mptr, osWaitForever);			//Post pointer to memory pool buffer
		osDelay(1000);
	}	
}
 
/*----------------------------------------------------------------------------
 mpool_consumer
 *---------------------------------------------------------------------------*/
void mpool_consumer(void const *argument)
{
	char msg[80] = {0};
  measurement1_st  *rptr;
  osEvent evt;
   
  for (;;)
	{
		evt = osMessageGet(msgQ_ID, osWaitForever);								//wait for message to arrive
		if (evt.status == osEventMessage)
		{																													//check we have received a message
			rptr = (measurement1_st*)evt.value.p;																			//read the pointer to memory pool buffer

			sprintf(msg, "\nmpool_consumer\n");
			USART_DMA_PutString((uint8_t*)msg);
			sprintf(msg, "Voltage: %.2f V\n", rptr->voltage);
			USART_DMA_PutString((uint8_t*)msg);
			sprintf(msg, "Current: %.2f A\n", rptr->current);
			USART_DMA_PutString((uint8_t*)msg);
			sprintf(msg, "Number of cycles: %d\n", rptr->counter);
			USART_DMA_PutString((uint8_t*)msg);
			
			//USART_PutString((uint8_t*)msg); 					// pour USART simple
			
//      printf("\nmpool_consumer\n");
//      printf("Voltage: %.2f V\n", rptr->voltage);
//      printf("Current: %.2f A\n", rptr->current);
//      printf("Number of cycles: %d\n", rptr->counter);
			
			osPoolFree(mpool_ID, rptr);															//Release the buffer
		}
  }
}
 
/*----------------------------------------------------------------------------
 mail_producer
 *---------------------------------------------------------------------------*/
void mail_producer(void const *argument)
{						
  measurement2_st *mptr;

	while (1) 
	{
		mptr = (measurement2_st*)osMailAlloc(mailBox_ID, osWaitForever);
		mptr->voltage = 1.23456789;
		mptr->current = 0.0987654321;
		mptr->counter = 66006;
		osMailPut(mailBox_ID, mptr); //Post the mail to the mailbox
		osDelay(1000);
	}
}

/*----------------------------------------------------------------------------
 mail_Consumer
 *---------------------------------------------------------------------------*/
void mail_consumer(void const *argument)
{
	char msg[80] = {0};
  measurement2_st *rptr;
	osEvent evt;																								//declare an osEvent variable
	
	while(1) 
	{
		evt = osMailGet(mailBox_ID, osWaitForever);								//wait until a message arrives
		if (evt.status == osEventMail) 														//Check for a valid message
		{
			rptr = (measurement2_st*)evt.value.p;
			
			sprintf(msg, "\nmail_consumer\n");
			USART_DMA_PutString((uint8_t*)msg);
			sprintf(msg, "Voltage: %.2f V\n", rptr->voltage);
			USART_DMA_PutString((uint8_t*)msg);
			sprintf(msg, "Current: %.2f A\n", rptr->current);
			USART_DMA_PutString((uint8_t*)msg);
			sprintf(msg, "Number of cycles: %d\n", rptr->counter);
			USART_DMA_PutString((uint8_t*)msg);
			
			//USART_PutString((uint8_t*)msg); 					// pour USART simple

//			printf("\nmail_consumer\n");
//			printf("Voltage: %.2f V\n", rptr->voltage);
//			printf("Current: %.2f A\n", rptr->current);
//			printf("Number of cycles: %d\n", rptr->counter);
			
			osMailFree(mailBox_ID, rptr);
		}
	}	
}

/*----------------------------------------------------------------------------
  Timer callback
 *---------------------------------------------------------------------------*/
void callbackTimer(void const *param)
{
	switch((uint32_t)param)
	{
		case 0:
			GPIOD->ODR ^= 0x2000;	// PD13, Led 3 (orange)
		break;
		
		case 1:
			GPIOD->ODR ^= 0x4000; // PD14, Led 5 (rouge)
		break;
		
		case 2:
			GPIOD->ODR ^= 0x8000; // PD15, Led 6 (bleu)
		break;

		case 3:
			GPIOD->ODR ^= 0x1000; // PD12, Led 4 (vert)
		break;
	}
}

/*----------------------------------------------------------------------------
  Task 2 'ledOn': switches the LED on
 *---------------------------------------------------------------------------*/
void msgQ_Thread2(void const *argument)
{
	for(;;) 
	{
		result = osMessageGet(Q_LED, osWaitForever);				// wait for a message to arrive
		LED_On(result.value.v);                          		// write the data to the LED's
		printf("On allume la LED#%d\n", result.value.v);
	}
}
/*----------------------------------------------------------------------------
  Task 1 'ledOff': switches the LED off
 *---------------------------------------------------------------------------*/
void msgQ_Thread1(void const *argument)
{
	for (;;) 
	{
		osMessagePut(Q_LED, 0x00, osWaitForever); // On demande à allumer la led 0
		osDelay(1000);
		printf("On éteind la LED#0\n");
		LED_Off(0x0);
		osMessagePut(Q_LED, 0x01, osWaitForever); // On demande à allumer la led 1
		osDelay(1000);
		printf("On éteind la LED#1\n");
		LED_Off(0x01);
		osMessagePut(Q_LED, 0x02, osWaitForever); // On demande à allumer la led 2
		osDelay(1000);
		printf("On éteind la LED#2\n");
		LED_Off(0x02);
		osMessagePut(Q_LED, 0x03, osWaitForever); // On demande à allumer la led 3
		osDelay(1000);
		printf("On éteind la LED#3\n");
		LED_Off(0x03);
	}
}

/*----------------------------------------------------------------------------
  Wait to acquire a semaphore token from sem1 then flash LED 1
 *---------------------------------------------------------------------------*/
void sem_Thread1(void const *argument)
{
	for (;;) 
	{
		osSemaphoreWait(sem1_ID, osWaitForever);
    printf("      0 ON\n");		
		LED_On(0);
		osSemaphoreWait(sem1_ID, osWaitForever);
    printf("0 OFF\n");		
		LED_Off(0);
	}
}

/*----------------------------------------------------------------------------
  Flash LED 2 and 'release' a semaphore token to sem1
 *---------------------------------------------------------------------------*/
void sem_Thread2(void const *argument)
{
	uint32_t r = 0;
	uint32_t min = 100;
	uint32_t max = 2000;
	for (;;) 
	{	
    printf("      1 ON\n");		
		LED_On(1);
		osSemaphoreRelease(sem1_ID);
		r = (TM_RNG_Get() % (max + 1 - min)) + min;
		osDelay(r);
    printf("1 OFF\n");		
		LED_Off(1);
		osSemaphoreRelease(sem1_ID);
		r = (TM_RNG_Get() % (max + 1 - min)) + min;
		osDelay(r);
	}
}

/*----------------------------------------------------------------------------
 multiplex_Thread
 *---------------------------------------------------------------------------*/
void multiplex_Thread(void const *argument)
{
	uint32_t r = 0;
	uint32_t min = 100;
	uint32_t max = 1000;
	for (;;) 
	{
    osSemaphoreWait(semMultiplex_ID, osWaitForever);
		
		r = (TM_RNG_Get() % (max + 1 - min)) + min;
		
		LED_On((uint32_t)argument);
		
		osDelay(r);
		
		LED_Off((uint32_t)argument);
		
		//osDelay(r);

		osSemaphoreRelease(semMultiplex_ID);
	}
}

/*----------------------------------------------------------------------------
 Thread 1 writes the character '1'
 *---------------------------------------------------------------------------*/
void send_usart2_Task(void const *argument)
{
#define RANDOM_USART2
	
#ifdef RANDOM_USART2
	uint32_t r = 0;
	uint32_t min = 500;
	uint32_t max = 2500;
#endif
	uint8_t msg[20] = "Hello, World !\n";
	for (;;) 
	{
		#ifdef RTE_COMPILER_EVENT_RECORDER
		EventRecord2(0x01, 0x00, 0x00);
		#endif	
		USART_DMA_PutString(msg);
#ifdef RANDOM_USART2
		r = (TM_RNG_Get() % (max + 1 - min)) + min;
		osDelay(r);
#else
			osDelay(1000);
#endif
	}
}
	

/*----------------------------------------------------------------------------
 Thread 1 writes the character '1'
 *---------------------------------------------------------------------------*/
void sendITM_Thread1(void const *argument)
{
#define MUTEX_ITM
#define RANDOM_ITM
	
#ifdef RANDOM_ITM
	uint32_t r = 0;
	uint32_t min = 1;
	uint32_t max = 100;
#endif
	uint32_t i;

	for (;;) 
	{
#ifdef MUTEX_ITM
		osMutexWait(sendITM_mutex_ID, osWaitForever); 
#endif
		for(i=0; i<20; i++)
		{
#ifdef RANDOM_ITM
			r = (TM_RNG_Get() % (max + 1 - min)) + min;
#endif
			sendCharITM('1');
#ifdef RANDOM_ITM
			osDelay(r);
#else
			osDelay(100);
#endif
		}
		sendCharITM('\n');
		sendCharITM('\r');
#ifdef MUTEX_ITM
		osMutexRelease(sendITM_mutex_ID);	
#endif
	}
}

/*----------------------------------------------------------------------------
  Thread 2 writes the character '2'
 *---------------------------------------------------------------------------*/
void sendITM_Thread2(void const *argument)
{
#ifdef RANDOM_ITM
	uint32_t r = 0;
	uint32_t min = 1;
	uint32_t max = 100;
#endif
	uint32_t i;
	for(;;)
	{
#ifdef MUTEX_ITM
		osMutexWait(sendITM_mutex_ID, osWaitForever);
#endif
		for(i=0; i<20; i++)
		{
#ifdef RANDOM_ITM
			r = (TM_RNG_Get() % (max + 1 - min)) + min;
#endif
			sendCharITM('2');
#ifdef RANDOM_ITM
			osDelay(r);
#else
			osDelay(100);
#endif
		}
		sendCharITM('\n');
		sendCharITM('\r');
#ifdef MUTEX_ITM
		osMutexRelease(sendITM_mutex_ID);
#endif
	}
}

/*----------------------------------------------------------------------------
 Thread 1 writes the character '1'
 *---------------------------------------------------------------------------*/
void sendUSART_Thread1(void const *argument)
{
#define MUTEX_USART
	
	uint32_t i, j = 0;
  //char *acMsg = "A11111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111B";
  //char *acMsg = "A111111111B";
  //char acBuffer[MAX_BUFFER_TX];
	for (;;) 
	{
#ifdef MUTEX_USART
			osMutexWait(sendUSART_mutex_ID, osWaitForever); 
#endif
		for(j=0; j<10; j++)
		{
			for(i=0; i<50; i++)
			{
				USART_PutChar('1');
				//osDelay(10);
				//sprintf(acBuffer, "%s(%d)\n", acMsg, i+1);
				//sendString((uint8_t *)acBuffer);
				//sendString((uint8_t *)acMsg);
			}
			USART_PutChar(0x0A);
			//osDelay(10);
			//sendString((uint8_t*)"\n");
		}
#ifdef MUTEX_USART
			osMutexRelease(sendUSART_mutex_ID);	
#endif
	}
}

/*----------------------------------------------------------------------------
  Thread 2 writes the character '2'
 *---------------------------------------------------------------------------*/
void sendUSART_Thread2(void const *argument)
{
	uint32_t i, j = 0;
  //char *acMsg = "C22222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222D";
  //char *acMsg = "C222222222D";
  //char acBuffer[MAX_BUFFER_TX];
	for(;;)
	{
#ifdef MUTEX_USART
			osMutexWait(sendUSART_mutex_ID, osWaitForever);
#endif
		for(j=0; j<10; j++)
		{
			for(i=0; i<50; i++)
			{
				USART_PutChar('2');
				//osDelay(10);
				//sprintf(acBuffer, "%s(%d)\n", acMsg, i+1);
				//sendString((uint8_t *)acBuffer);
				//sendString((uint8_t *)acMsg);
			}
			USART_PutChar(0x0A);
			//osDelay(10);
			//sendString((uint8_t *)"\n");
		}
#ifdef MUTEX_USART
			osMutexRelease(sendUSART_mutex_ID);
#endif
	}
}

// ************************************************ ******************************
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  SystemCoreClockUpdate();

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

	LED_Initialize();

  TM_RNG_Init();
	
  //rx_data = 0x00;
  // Déclenche l'interruption sur l'arrivée d'un caractère
  // Attention de ne pas placer cette ligne après osKernelStart()
  //HAL_UART_Receive_IT(&huart2, &rx_data, 1);
	HAL_UART_Receive_DMA(&huart2, rx_buffer, 1); 

#ifdef RTE_COMPILER_EVENT_RECORDER
		EventRecorderInitialize(EventRecordAll, 1);
#endif
	
	printf("C'est parti !\n");
	
  /* Initialize CMSIS-RTOS */
  osKernelInitialize ();

//	T_Simple_View_Thread = osThreadCreate(osThread(Simple_View_Thread), NULL);

//	T_Complex_View_Thread = osThreadCreate(osThread(Complex_View_Thread), NULL);	

//	// ---------- Transmission d'un message via usart2 à 9600bauds
//	T_send_usart2 = osThreadCreate(osThread(send_usart2_Task), NULL);
//	
//	// ---------- Mutex avec sortie USART
//	sendUSART_mutex_ID = osMutexCreate(osMutex(sendUSART_mutex));
//	T_sendUSART1 = osThreadCreate(osThread(sendUSART_Thread1), NULL);
//	T_sendUSART2 = osThreadCreate(osThread(sendUSART_Thread2), NULL);
//	
//	// ---------- Mutex avec sortie ITM
//	sendITM_mutex_ID = osMutexCreate(osMutex(sendITM_mutex));
//	T_sendITM1 = osThreadCreate(osThread(sendITM_Thread1), NULL);
//	T_sendITM2 = osThreadCreate(osThread(sendITM_Thread2), NULL);
//	
//	// ---------- Multiplex (plusieurs instances d'un même thread avec passage de paramètre)
//	semMultiplex_ID = osSemaphoreCreate(osSemaphore(semMultiplex), 1);	
//	T_mux1 = osThreadCreate(osThread(multiplex_Thread),(void *)0);
//	T_mux2 = osThreadCreate(osThread(multiplex_Thread),(void *)1);                  
//	T_mux3 = osThreadCreate(osThread(multiplex_Thread),(void *)2);
//	T_mux4 = osThreadCreate(osThread(multiplex_Thread),(void *)3);
//	
		// ---------- Semaphore, Moi d'abord, Toi ensuite
	//sem_moi_d_abord_ID = osSemaphoreCreate(osSemaphore(sem_moi_d_abord), 0); // Le sémaphore sera créé dans le thread "moi_d_abord"
//	T_moi_d_abord = osThreadCreate(osThread(moi_d_abord), NULL);
//	T_toi_ensuite= osThreadCreate(osThread(toi_ensuite), NULL);

		// ---------- Symmetry
//	sem_Symmetry_ID = osSemaphoreCreate(osSemaphore(sem_Symmetry), 0);
//	T_Task1_Symmetry = osThreadCreate(osThread(Task1_Symmetry), NULL);
//	T_Task2_Symmetry = osThreadCreate(osThread(Task2_Symmetry), NULL);

//	// ---------- Semaphore
//	sem1_ID = osSemaphoreCreate(osSemaphore(sem1), 0);
//	T_semLedOn = osThreadCreate(osThread(sem_Thread1), NULL);
//	T_semLedOff = osThreadCreate(osThread(sem_Thread2), NULL);

	// ---------- Message Queue
//	Q_LED = osMessageCreate(osMessageQ(Q_LED), NULL);
//	T_msgQ_Thread1 = osThreadCreate(osThread(msgQ_Thread1), NULL);
//	T_msgQ_Thread2 = osThreadCreate(osThread(msgQ_Thread2), NULL);

//	// ---------- Timer virtuel
//	timer0_ID = osTimerCreate(osTimer(timer0_handle), osTimerPeriodic, (void *)0);	
//	timer1_ID = osTimerCreate(osTimer(timer1_handle), osTimerPeriodic, (void *)1);	
//	timer2_ID = osTimerCreate(osTimer(timer2_handle), osTimerPeriodic, (void *)2);	
//	timer3_ID = osTimerCreate(osTimer(timer3_handle), osTimerPeriodic, (void *)3);	
//	osTimerStart(timer0_ID, 250);	
//	osTimerStart(timer1_ID, 500);	
//	osTimerStart(timer2_ID, 1000);	
//	osTimerStart(timer3_ID, 2000);

//	// ---------- Mail box
//	mailBox_ID = osMailCreate(osMailQ(mailBox), NULL);
//  T_mail_producer = osThreadCreate(osThread(mail_producer), NULL);
//	T_mail_consumer = osThreadCreate(osThread(mail_consumer), NULL);

//	// ---------- Memory Pool
//  mpool_ID = osPoolCreate(osPool(mpool));                 						// create memory pool
//  msgQ_ID = osMessageCreate(osMessageQ(msgQ), NULL);  								// create msg queue  
//  T_mpool_producer = osThreadCreate(osThread(mpool_producer), NULL);
//  T_mpool_consumer = osThreadCreate(osThread(mpool_consumer), NULL);

//	// ---------- Signal
//	T_signal1 = osThreadCreate(osThread(signal_Thread1), NULL);	
//	T_signal2 = osThreadCreate(osThread(signal_Thread2), NULL);

//	// ---------- Algorithme "Rendez-vous"
//	semArrived1_ID = osSemaphoreCreate(osSemaphore(semArrived1), 0);				
//	semArrived2_ID = osSemaphoreCreate(osSemaphore(semArrived2), 0);		
//	T_rendez_vous1 = osThreadCreate(osThread(rendez_vous_Thread1), NULL);
//	T_rendez_vous2 = osThreadCreate(osThread(rendez_vous_Thread2), NULL);

//	// ---------- Algorithme "Barrier"
//	semTurnstile_ID = osSemaphoreCreate(osSemaphore(semTurnstile), 0);
//	semTurnstile2_ID = osSemaphoreCreate(osSemaphore(semTurnstile2), 1);
//	mutexTurnstile_ID = osSemaphoreCreate(osSemaphore(mutexTurnstile), 1);	
//	T_turnstile_task0 = osThreadCreate(osThread(turnstile_threadBaseCode),(void *) 0x00);
//	T_turnstile_task1 = osThreadCreate(osThread(turnstile_threadBaseCode),(void *) 0x01); 
//	T_turnstile_task2 = osThreadCreate(osThread(turnstile_threadBaseCode),(void *) 0x02);
//	T_turnstile_task3 = osThreadCreate(osThread(turnstile_threadBaseCode),(void *) 0x03);

//	// ---------- Flashing LED threads with parameters
// Petit commentaire au passage
// Second commentaire
	T_flashing_LED_0 = osThreadCreate(osThread(flashing_LED_0), (void *) 0x01);
//	T_flashing_LED_1 = osThreadCreate(osThread(flashing_LED_1), (void *) 0x01);
//	T_flashing_LED_2 = osThreadCreate(osThread(flashing_LED_2), (void *) 0x01);
//	T_flashing_LED_3 = osThreadCreate(osThread(flashing_LED_3), (void *) 0x01);

  osKernelStart();
	osDelay(osWaitForever);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}
