/*
 * delay.c
 *
 *  Created on: Jun 1, 2018
 *      Author: Lyden
 */

#include "stm32f4xx_rcc.h"

/* 16 MHz internal clock */
/*#define DELAY_TIM_PRESCALER_US (16 - 1)
#define DELAY_TIM_PRESCALER_MS (16000 - 1)*/

/* 100 MHz internal clock */
#define DELAY_TIM_PRESCALER_US (100 - 1)
#define DELAY_MULTIPLIER_US (1)
#define DELAY_TIM_PRESCALER_MS (50000 - 1)
#define DELAY_MULTIPLIER_MS (2)

static void configure_delay_timer() {
	RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;
	TIM9->CR1 |= TIM_CR1_OPM; /* One pulse mode and reload every time */
	TIM9->EGR |= TIM_EGR_UG; /* Generate update event to load buffered values */

	/* Wait for update event flag to raise */
	while(!(TIM9->SR & TIM_SR_UIF));

	TIM9->SR &= ~TIM_SR_UIF; /* Clear the update event flag */
}

static inline void delay_prescaler(uint16_t delay, uint16_t prescaler) {
	/* Wait for timer to become available */
	while((TIM9->CR1 & TIM_CR1_CEN));

	/* Load delay and prescaler into timer */
	TIM9->ARR = delay;
	TIM9->PSC = prescaler;
	TIM9->EGR |= TIM_EGR_UG; /* Generate an update event */

	/* Wait for update event flag to raise */
	while(!(TIM9->SR & TIM_SR_UIF));

	TIM9->SR &= ~TIM_SR_UIF; /* Clear the update event flag */
	TIM9->CR1 |= TIM_CR1_CEN; /* Start timer */

	/* Wait for update event flag to raise */
	while(!(TIM9->SR & TIM_SR_UIF));

	TIM9->SR &= ~TIM_SR_UIF; /* Clear the update event flag */

}

extern void init_delay() {
	configure_delay_timer();
}

extern void delay_10ns(uint16_t delay) {
	delay_prescaler(delay, 0);
}

extern void delay_us(uint16_t delay) {
	delay_prescaler(delay * DELAY_MULTIPLIER_US, DELAY_TIM_PRESCALER_US);
}

extern void delay_ms(uint16_t delay) {
	delay_prescaler(delay * DELAY_MULTIPLIER_MS, DELAY_TIM_PRESCALER_MS);
}
