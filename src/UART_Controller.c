/*
 * UART.c
 *
 *  Created on: Jun 8, 2018
 *      Author: Poornachander
 */


#include <string.h>
#include "stm32f4xx_dma.h"
#include "FreeRTOS.h"
#include "Task.h"
#include "Buffer.h"
#include "UART_Controller.h"

//Register bit for enabling TXEIE bit. This is used instead of the definitions in stm32f4xx_usart.h
#define USART_TXEIE	0b10000000
#define USART_RXEIE	0b100000

// Receive buffer for UART, no DMA
char inputString[MAX_BUFFER_DATA]; //string to store individual bytes as they are sent
uint8_t inputStringIndex = 0;

// Receive Buffer to send to Command Handler
Buffer_t inputBuffer;

// Task handle to notify FSM task
TaskHandle_t UARTTaskToNotify = NULL;

static void Configure_GPIO_USART2(void) {
	/* Enable the peripheral clock of GPIOA */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;

	/* GPIOA Configuration: TIM5 CH1 (PA0) */
	GPIO_InitStructure.GPIO_Pin |= GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; // Input/Output controlled by peripheral
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Connect USART1 pins to AF */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
}

/**
 * @brief  This function configures USART1.
 * @param  None
 * @retval None
 */
static void Configure_USART2(void) {
	/* Enable the peripheral clock USART1 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	//RCC->CFGR3 |= RCC_CFGR3_USART1SW_1;
	USART_InitTypeDef USART_InitStruct; // this is for the USART1 initialization

	USART_InitStruct.USART_BaudRate = 9600;	// the baudrate is set to the value we passed into this init function
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;// we want the data frame size to be 8 bits (standard)
	USART_InitStruct.USART_StopBits = USART_StopBits_1;	// we want 1 stop bit (standard)
	USART_InitStruct.USART_Parity = USART_Parity_No;// we don't want a parity bit (standard)
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
	USART_Init(USART2, &USART_InitStruct);

	USART2->CR1 |= USART_RXEIE; //Enable the USART1 receive interrupt

	/* Configure IT */
	/* (3) Set priority for USART1_IRQn */
	/* (4) Enable USART1_IRQn */
	NVIC_SetPriority(USART2_IRQn, 7); /* (3) */
	NVIC_EnableIRQ(USART2_IRQn); /* (4) */

	// finally this enables the complete USART1 peripheral
	USART_Cmd(USART2, ENABLE);
}

static void Configure_DMA_USART2() {


	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

	//Initialize DMA USing DMA1 Stream6 Channel 4
	DMA1_Stream6->CR = 0;

	DMA1_Stream6->PAR = (uint32_t) &USART2->DR;
	DMA1_Stream6->CR |= DMA_Channel_4; //Set Channel 4
	DMA1_Stream6->CR |= DMA_Priority_Medium; // Set priority to medium
	DMA1_Stream6->CR |= DMA_DIR_MemoryToPeripheral; //Set direction, memory-to-peripheral
	DMA1_Stream6->CR |= DMA_MemoryInc_Enable; //Set memory increment mode
	DMA1_Stream6->CR |= DMA_IT_TC; // //Transfer complete interrupt enable

	// Enable DMA Transmit in the USART control register
	USART2->CR3 |= USART_DMAReq_Tx;

	NVIC_InitTypeDef NVIC_InitStructure;
	//Enable DMA2 channel IRQ Channel */
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}

extern int8_t UART_push_out_len(char* mesg, uint16_t len) {

	if((DMA1_Stream6->CR & 0x1) == 0) {

		DMA1_Stream6->NDTR = len;
		DMA1_Stream6->M0AR = (uint32_t)mesg;
		DMA1_Stream6->CR |= 0x1;
	}

}

extern void UART_init(TaskHandle_t currentHandle) {

	//initialize the input buffer
	Buffer_init(&inputBuffer);

	//Get current task handle
	UARTTaskToNotify = currentHandle;

	//initialize the UART driver
	Configure_GPIO_USART2();
	Configure_USART2();
	Configure_DMA_USART2();
}


// This is handling two cases. The interrupt will run if a character is received
// and when a transmission is completed
void USART2_IRQHandler() {

	if((USART2->SR & USART_FLAG_RXNE) == USART_FLAG_RXNE) { //If character is received

			char tempInput[1];
			tempInput[0] = USART2->DR;

			//Check for new line character which indicates end of command
			if (tempInput[0] == '\n' || tempInput[0] == '\r') {

				if(strlen(inputString) > 0) {
					Buffer_add(&inputBuffer, inputString, MAX_BUFFER_DATA);
					memset(inputString, 0, MAX_BUFFER_DATA);
					inputStringIndex = 0;

					BaseType_t xHigherPriorityTaskWoken = pdFALSE;
					vTaskNotifyGiveFromISR(UARTTaskToNotify, &xHigherPriorityTaskWoken);
				}

			} else {
				inputString[inputStringIndex] = tempInput[0];
				inputStringIndex = (inputStringIndex + 1) & (MAX_BUFFER_DATA -1);
			}

		}

}

void DMA1_Stream6_IRQHandler() {

	DMA1->HIFCR |= DMA_HIFCR_CTCIF6; //Clear TC interrupt flag

}
