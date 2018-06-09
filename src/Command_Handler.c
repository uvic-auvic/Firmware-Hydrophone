/*
 * UART_Command_Handler.c
 *
 *  Created on: Jun 8, 2018
 *      Author: Poornachander
 */

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "UART_Controller.h"
#include "Buffer.h"
#include "ADC.h"

void Command_Handler() {

	char commandString[MAX_BUFFER_SIZE];

	while(1) {
		//it's important that this is while, if the task is accidentally awaken it
		//can't execute without having at least one item the input buffer
		while(inputBuffer.size == 0){

			//sleeps the task until it is notified by the UART controller
			ulTaskNotifyTake( pdTRUE, portMAX_DELAY);

		}

		Buffer_pop(&inputBuffer, commandString);

		if(strncmp(commandString, "SEND", 4) == 0)
		{
			transmit_ADC_readings();
		}

		else if(strncmp(commandString, "SZ?", 3) == 0)
		{
			uint16_t size = get_ADC_buffer_size();
			char* end_string = "\r\n";
			UART_push_out_len((char*)&size, 2);
			UART_push_out_len(end_string, 2);
		}

	}

}

extern void UART_Command_Handler_init() {

	TaskHandle_t xHandle = NULL;

	xTaskCreate(
			Command_Handler,       /* Function that implements the task. */
			(const char *) "Command_Handler",          /* Text name for the task. */
			400,      /* Stack size in words, not bytes. */
			NULL,    /* Parameter passed into the task. */
			tskIDLE_PRIORITY + 1,/* Priority at which the task is created. */
			&xHandle );      /* Used to pass out the created task's handle. */

	UART_init(xHandle);

}
