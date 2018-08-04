/*
 * UART_Command_Handler.c
 *
 *  Created on: Jun 8, 2018
 *      Author: Poornachander
 */

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "semphr.h"
#include "UART_Controller.h"
#include "Buffer.h"
#include "ADC.h"
#include "pinger_detection.h"

#define MAX_PACKET_SIZE (4080)
#define DEVICE_ID ("Hydrophones v1.0\r\n\0")
#define MAX_OUTPUT_SIZE (32)

SemaphoreHandle_t  data_sending = NULL;

typedef struct Message_Header {
	uint32_t CRC32;
	char command[2];
	uint8_t packet_count;
	uint8_t packet_idx;
	uint16_t packet_size;
	uint16_t total_data_size;
} Message_Header_t;

static int asciiToInt(char input[], uint8_t length) {

	int output = 0;

	for(int i = 0; i < length; i++) {

		if (input[i] >= '0' && input[i] <= '9') {
			output = output * 10;
			output += (int)(input[i] - '0');
		} else {
			return -1; //Error: ASCII character is not a number
		}
	}

	return output;
}

static inline uint32_t reverse_bit_order(uint32_t input_word) {
	uint32_t output_word = input_word;
	output_word = ((output_word >>  1) & 0x55555555) | ((output_word <<  1) & 0xaaaaaaaa);
	output_word = ((output_word >>  2) & 0x33333333) | ((output_word <<  2) & 0xcccccccc);
	output_word = ((output_word >>  4) & 0x0f0f0f0f) | ((output_word <<  4) & 0xf0f0f0f0);
	output_word = ((output_word >>  8) & 0x00ff00ff) | ((output_word <<  8) & 0xff00ff00);
	output_word = ((output_word >> 16) & 0x0000ffff) | ((output_word << 16) & 0xffff0000);
	return output_word;
}

/* Calculates the CRC32 result of the data with 0xFFFFFFFF starting value.
 * 	Inputs: - data: pointer to the start of the data
 * 			- data_size: size of data, in bytes
 * 	Output: - CRC32 result of data
 */
static uint32_t CRC32(uint32_t* data, uint16_t data_size) {
	uint16_t word_size = data_size / 4;
	uint8_t extra_bytes = data_size % 4;

	/* Reset the CRC unit */
	CRC->CR = CRC_CR_RESET;

	/* Compute CRC of data */
	for(int i = 0; i < word_size; ++i) {
		CRC->DR = *data++;
	}

	/* Return the result */
	return CRC->DR;
}

void Command_Handler() {

	char commandString[MAX_BUFFER_SIZE];
	char outputString[MAX_OUTPUT_SIZE];
	Message_Header_t ADC_message_header;
	uint16_t packet_size = MAX_PACKET_SIZE;
	uint8_t* ADC_Buffer_ptr = get_ADC_buffer();

	/* Initialize ADC data read message header */
	strncpy((char*)ADC_message_header.command, "DR", 2);
	ADC_message_header.total_data_size = get_ADC_buffer_size();
	ADC_message_header.packet_size = packet_size - (packet_size % 4);
	ADC_message_header.packet_count = ADC_message_header.total_data_size / ADC_message_header.packet_size;
	if(ADC_message_header.total_data_size % ADC_message_header.packet_size) {
		ADC_message_header.packet_count++;
	}
	ADC_message_header.packet_idx = 0;

	data_sending = xSemaphoreCreateMutex();

	while(1) {
		//it's important that this is while, if the task is accidentally awaken it
		//can't execute without having at least one item the input buffer
		while(inputBuffer.size == 0){

			//sleeps the task until it is notified by the UART controller
			ulTaskNotifyTake( pdTRUE, portMAX_DELAY);

		}

		Buffer_pop(&inputBuffer, commandString);

		/* ---- Check for different commands ---- */

		/* Command to return ID of the device */
		if(strncmp(commandString, "RID", 3) == 0) {
			strncpy(outputString, DEVICE_ID, strlen(DEVICE_ID));
			UART_push_out_len(outputString, strlen(DEVICE_ID));
		}

		/* Command to send the next packet in ADC conversions */
		else if(strncmp(commandString, "NEXT", 4) == 0) {
			/* Increment the packet index */
			ADC_message_header.packet_idx++;

			/* Check if we're sending the last packet */
			if(ADC_message_header.packet_idx == (ADC_message_header.packet_count - 1)) {
				ADC_message_header.packet_size =
					ADC_message_header.total_data_size - packet_size * ADC_message_header.packet_idx;
			}

			/* Check if we're done (send nothing but the header if we are) */
			else if(ADC_message_header.packet_idx == ADC_message_header.packet_count) {
				ADC_message_header.packet_size = 0;
				xSemaphoreGive(data_sending);
			}

			/* Compute the CRC */
			ADC_message_header.CRC32 = CRC32(
					(uint32_t*)&(ADC_Buffer_ptr[ADC_message_header.packet_idx * ADC_message_header.packet_size]),
					ADC_message_header.packet_size);

			/* Send out the header */
			UART_push_out_len((char*)&ADC_message_header, sizeof(Message_Header_t));

			/* Send out the ADC data */
			if(ADC_message_header.packet_size) {
				UART_push_out_len(
					(char*)&(ADC_Buffer_ptr[ADC_message_header.packet_idx * ADC_message_header.packet_size]),
					ADC_message_header.packet_size);
			}
		}

		/* Command to resend last packet */
		else if(strncmp(commandString, "RETRY", 5) == 0) {
			/* Send out the header */
			UART_push_out_len((char*)&ADC_message_header, sizeof(Message_Header_t));

			/* Send out the ADC data */
			if(ADC_message_header.packet_size) {
				UART_push_out_len(
					(char*)&(ADC_Buffer_ptr[ADC_message_header.packet_idx * ADC_message_header.packet_size]),
					ADC_message_header.packet_size);
			}
		}

		/* Command to start ADC converions and send first packet */
		else if(strncmp(commandString, "ADCDR", 5) == 0) {
//			/* Start and complete conversions */
//			complete_ADC_conversions();

			xSemaphoreTake(data_sending, 500);
			memcpy(ADC_Buffer, detected_data, (4096 * 4) + 1);

			/* Start at packet 0 */
			ADC_message_header.packet_idx = 0;

			/* Initialize packet_size */
			ADC_message_header.packet_size = packet_size;

			/* Check to see if we're on the last packet */
			if(ADC_message_header.packet_idx == (ADC_message_header.packet_count - 1)) {
				ADC_message_header.packet_size =
					ADC_message_header.total_data_size - ADC_message_header.packet_size * ADC_message_header.packet_idx;
			}

			/* Compute the CRC */
			ADC_message_header.CRC32 = CRC32((uint32_t*)ADC_Buffer_ptr, ADC_message_header.packet_size);

			/* Send out the header */
			UART_push_out_len((char*)&ADC_message_header, sizeof(Message_Header_t));

			/* Send out the ADC data */
			if(ADC_message_header.packet_size) {
				UART_push_out_len((char*)ADC_Buffer_ptr, ADC_message_header.packet_size);
			}
		}

		/* ADC Buffer size query in bytes */
		else if(strncmp(commandString, "DS?", 3) == 0) {
			uint16_t size = get_ADC_buffer_size();
			memcpy(outputString, &size, 2);
			strncpy(&(outputString[2]), "\r\n", 2);
			UART_push_out_len(outputString, 4);
		}

		/* Set ADC Buffer size. Returns new size */
		else if(strncmp(commandString, "DS", 2) == 0 && (strnlen(commandString, 7) == 7)) {
			int data_size = asciiToInt(&(commandString[2]), 5);
			if(data_size < 0) {
				memcpy(outputString, &ADC_message_header.total_data_size, 2);
				strncpy(&(outputString[2]), "\r\n", 2);
				UART_push_out_len(outputString, 4);
			}
			else {
				/* Make sure data isn't too big */
				if(data_size > (0xFF * ADC_message_header.packet_size)) {
					data_size = 0xFF * ADC_message_header.packet_size;
				}

				/* Update ADC buffer size */
				uint16_t set_size = set_ADC_buffer_size(data_size);

				/* Update packet header */
				ADC_message_header.total_data_size = set_size;
				ADC_message_header.packet_count = set_size / ADC_message_header.packet_size;
				if(set_size % ADC_message_header.packet_size) {
					ADC_message_header.packet_count++;
				}

				/* Return set size */
				memcpy(outputString, &set_size, 2);
				strncpy(&(outputString[2]), "\r\n", 2);
				UART_push_out_len(outputString, 4);
			}
		}

		/* Query packet size */
		else if(strncmp(commandString, "PS?", 3) == 0) {
			memcpy(outputString, &ADC_message_header.packet_size, 2);
			strncpy(&(outputString[2]), "\r\n", 2);
			UART_push_out_len(outputString, 4);
		}

		/* Set packet size. Return set packet size */
		else if(strncmp(commandString, "PS", 2) == 0 && (strnlen(commandString, 7) == 7)) {
			int poss_packet_size = asciiToInt(&(commandString[2]), 5);

			/* Make sure packet size is a multiple of 4 */
			poss_packet_size -= (poss_packet_size % 4);

			/* Make sure packet size isn't too large */
			if(poss_packet_size > MAX_PACKET_SIZE) {
				poss_packet_size = MAX_PACKET_SIZE;
			}

			/* Make sure packet size isn't invalid */
			if(poss_packet_size <= 0) {
				poss_packet_size = ADC_message_header.packet_size;
			}

			/* Make sure packet size isn't too small */
			if(poss_packet_size < (ADC_message_header.total_data_size / 0xFF)) {
				poss_packet_size = ADC_message_header.total_data_size / 0xFF;
			}

			/* Update the ADC message header */
			packet_size = poss_packet_size;
			ADC_message_header.packet_size = packet_size;
			ADC_message_header.packet_count = ADC_message_header.total_data_size / packet_size;
			if(ADC_message_header.total_data_size % packet_size) {
				ADC_message_header.packet_count++;
			}

			/* Send response to host */
			memcpy(outputString, &packet_size, 2);
			strncpy(&(outputString[2]), "\r\n", 2);
			UART_push_out_len(outputString, 4);
		}

		else if(strncmp(commandString, "EGTH", 4) == 0 && strlen(commandString) == 8) {

			energy_threshold = (uint16_t)asciiToInt(&commandString[4], 4);
			char output[5] = {};
			itoa(energy_threshold, output, 10);
			UART_push_out_len(output, 4);
		}

		/* Not a valid command. Return Error. */
		else {
			strncpy(outputString, "ERROR\r\n", 7);
			UART_push_out_len(outputString, 7);
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

	/* Enable CRC clock */
	RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;
}
