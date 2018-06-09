/*
 * ADC.h
 *
 *  Created on: Jun 1, 2018
 *      Author: Lyden
 */

#ifndef ADC_H_
#define ADC_H_

extern void init_ADC();
extern void start_ADC_conversions();
extern void transmit_ADC_readings();
extern uint16_t get_ADC_buffer_size();

#endif /* ADC_H_ */
