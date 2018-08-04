/*
 * pinger_detection.c
 *
 *  Created on: Aug 3, 2018
 *      Author: Poornachander
 */
#include <stdlib.h>
#include <math.h>
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "semphr.h"

#include "ADC.h"
#include "Command_Handler.h"

#define ADC_center_value (2047)
uint16_t energy_threshold = 200;
uint16_t detected_data[(4096 * 4) + 1] = {};

void pinger_detection() {

	uint32_t signal_energy = 0;
	while(1) {

		if ( xSemaphoreTake(data_sending, 30000) == pdTRUE) {

			complete_ADC_conversions();

			for(uint16_t i = 0; i < 409; i = i + 4) {
				signal_energy += (abs((ADC_Buffer[i] - ADC_center_value))) / 2048;
			}

			if(signal_energy > energy_threshold) {
				memcpy(detected_data, ADC_Buffer, (4096 * 4) + 1);
			}

			signal_energy = 0;
			xSemaphoreGive(data_sending);
		}

		vTaskDelay(1);
	}

}

void init_pinger_detection() {

	xTaskCreate(pinger_detection,
		(const char * const)"pinger_detection",
		configMINIMAL_STACK_SIZE,
		NULL,                 // pvParameters
		tskIDLE_PRIORITY + 1, // uxPriority
		NULL              ); // pvCreatedTask */

}
