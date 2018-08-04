/*
 * UART_Command_Handler.h
 *
 *  Created on: Jun 8, 2018
 *      Author: Poornachander
 */

#ifndef COMMAND_HANDLER_H_
#define COMMAND_HANDLER_H_

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "semphr.h"

extern SemaphoreHandle_t  data_sending;
extern void UART_Command_Handler_init();

#endif /* COMMAND_HANDLER_H_ */
