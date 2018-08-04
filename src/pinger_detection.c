/*
 * pinger_detection.c
 *
 *  Created on: Aug 3, 2018
 *      Author: Poornachander
 */
//#include <stdlib.h>
//#include <math.h>
//#include <string.h>
//#include "stm32f4xx.h"
//#include "FreeRTOS.h"
//#include "FreeRTOSConfig.h"
//#include "task.h"
//#include "semphr.h"
//
//#include "ADC.h"
//#include "Command_Handler.h"
//
//#define ADC_center_value (32767)
//
//uint16_t energy_threshold = 2000;
//uint16_t detected_data[(4096 * 4) + 1] = {};

void pinger_detection() {
//
//	data_sending = xSemaphoreCreateMutex();
//
//	detected_data[0] = 5000;
//	detected_data[1] = 5000;
//	detected_data[2] = 5000;
//
//	float signal_energy = 0;
//	while(1) {
//
//		if ( xSemaphoreTake(data_sending, 30000) == pdTRUE) {
//
//			GPIOB->ODR ^= GPIO_Pin_12;
//
//			complete_ADC_conversions();
//
//			for(uint16_t i = 1; i < ((4096 * 4) + 1); i = i + 4) {
//				signal_energy += (float)(abs((int16_t)(ADC_Buffer[i] - (int16_t)ADC_center_value))) / 65536;
//			}
//
//			if(signal_energy > energy_threshold) {
//				memcpy(detected_data, ADC_Buffer, (4096 * 4) + 1);
//			}
//
//			signal_energy = 0;
//			xSemaphoreGive(data_sending);
//		}
//
//		vTaskDelay(100);
//	}

}

extern void init_pinger_detection() {

//	xTaskCreate(pinger_detection,
//		(const char * const)"pinger_detection",
//		configMINIMAL_STACK_SIZE,
//		NULL,                 // pvParameters
//		tskIDLE_PRIORITY + 1, // uxPriority
//		NULL              ); // pvCreatedTask */
//
//	/* Configure blinky LED for output mode */
//	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; /* Enable clock for B pins */
//	GPIOB->MODER |= GPIO_MODER_MODER12_0; /* Output mode */

}
