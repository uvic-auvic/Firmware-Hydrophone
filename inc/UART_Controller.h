/*
 * UART_Controller.h
 *
 *  Created on: Jun 8, 2018
 *      Author: Poornachander
 */

#ifndef UART_CONTROLLER_H_
#define UART_CONTROLLER_H_

#include "FreeRTOS.h"
#include "task.h"
#include "Buffer.h"

extern Buffer_t inputBuffer;

extern void UART_init(TaskHandle_t currentHandle);
extern int8_t UART_push_out_len(char* mesg, uint16_t len);

#endif /* UART_CONTROLLER_H_ */
