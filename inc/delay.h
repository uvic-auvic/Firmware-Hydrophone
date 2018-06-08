/*
 * delay.h
 *
 *  Created on: Jun 1, 2018
 *      Author: Lyden
 */

#ifndef DELAY_H_
#define DELAY_H_

extern void init_delay();
extern void delay_us(uint16_t delay);
extern void delay_ms(uint16_t delay);
extern void delay_ticks(uint16_t delay);

#endif /* DELAY_H_ */
