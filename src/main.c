/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/

#include "stm32f4xx.h"
#include "UART.h"
#include "DMA.h"
#include "ADC.h"
#include "Timer.h"
#include "main.h"

volatile uint16_t fft_buffer[ARRAYSIZE];
extern volatile uint8_t dma_state;

int main(void)
{
	//ADC_init();
	UART_init();
	timer_init();
	DMA_init();
	//Start ADC1 Software Conversion
	//ADC_SoftwareStartConv(ADC1);

	while (1)
	{
		if(dma_state > 0){
			memcpy(fft_buffer, "Hello world!\n", 14);
			while(dma_state != 2);
		}
	}
}



