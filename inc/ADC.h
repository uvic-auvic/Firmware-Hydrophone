/*
 * ADC.h
 *
 *  Created on: Jun 1, 2018
 *      Author: Lyden
 */

#ifndef ADC_H_
#define ADC_H_

#include <stdint.h>

#define ADC_MAX_READINGS (4096)
#define ADC_CHANNEL_COUNT (4)
#define ADC_BUFFER_SIZE (ADC_MAX_READINGS * ADC_CHANNEL_COUNT)

extern uint16_t ADC_Buffer[ADC_BUFFER_SIZE + 1];
extern uint16_t buffer_size;
extern volatile uint8_t ADC_mutex;

extern void init_ADC();
extern void start_ADC_conversions();
extern uint16_t get_ADC_buffer_size();
extern uint16_t set_ADC_buffer_size(uint16_t size);
extern uint8_t* get_ADC_buffer();
extern void complete_ADC_conversions();

#endif /* ADC_H_ */
