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
#include "main.h"

int main(void)
{
	ADC_init();
	UART_init();
	DMA_init();
	//Start ADC1 Software Conversion
	ADC_SoftwareStartConv(ADC1);


	while (1)
	{

	}
}



