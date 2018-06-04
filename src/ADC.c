/*
 * ADC.c
 *
 *  Created on: Jun 1, 2018
 *      Author: Lyden
 */

#include "stm32f4xx_rcc.h"
#include "delay.h"

#define GPIO_AFRH_AFRH8_0 (0x00000001)
#define GPIO_AFRH_AFRH8_1 (0x00000002)
#define GPIO_AFRH_AFRH8_2 (0x00000004)
#define GPIO_AFRH_AFRH8_3 (0x00000008)

#define GPIO_AFRL_AFRL4_0 (0x00010000)
#define GPIO_AFRL_AFRL4_1 (0x00020000)
#define GPIO_AFRL_AFRL4_2 (0x00040000)
#define GPIO_AFRL_AFRL4_3 (0x00080000)

/* Scale down to 10MHz */
/* 50% Duty Cycle (x >> 1 => x divided by 2) */
#define EXT_CLOCK_PERIOD (10 - 1) /* Count values: {0,1,2,3,4,5,6,7,8,9} */
#define EXT_CLOCK_DUTY ((EXT_CLOCK_PERIOD + 1) >> 1) /* High for {0,1,2,3,4}, Low for {5,6,7,8,9} */

/* CONVST PWM with computed period and a on time of 37 cycles */
#define CONVERSION_TIME (37)
#define ACQUISITION_TIME (10)
#define CONVST_PERIOD (((CONVERSION_TIME + ACQUISITION_TIME) * (EXT_CLOCK_PERIOD + 1)) - 1)
#define CONVST_DUTY (CONVERSION_TIME * (EXT_CLOCK_PERIOD + 1))

/* RD PWM with 50% duty, 3.33 MHz frequency */
#define RD_PERIOD ((3 * (EXT_CLOCK_PERIOD + 1)) - 1)
#define RD_DUTY ((RD_PERIOD + 1) >> 1)

static void configure_ADC_GPIO() {
	/* Configure CS (Chip Select) Pin as output pin PB0 and set it low */
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; /* Enable clock for B pins */
	GPIOB->MODER |= GPIO_MODER_MODER0_0; /* Output mode */
	GPIOB->BSRRH = GPIO_BSRR_BR_0 >> 16; /* Set pin to 0 */

	/* Configure RD (Read) Pin as output pin PB1 and set it high */
	GPIOB->MODER |= GPIO_MODER_MODER1_0;
	GPIOB->BSRRL = GPIO_BSRR_BS_1;

	/* Configure WR (Write) Pin as output pin PB2 and set it high */
	GPIOB->MODER |= GPIO_MODER_MODER2_0; /* Output mode */
	GPIOB->BSRRL = GPIO_BSRR_BS_2;

	/* CONVST is on PB4 and is configured in configure_CONVST_PWM function */

	/* EOLC is on PB7 and is an input pin by default */

	/* Configure CLK_CNTRL pin as output PB10 and set it low for now */
	GPIOB->MODER |= GPIO_MODER_MODER10_0; /* Output mode */
	GPIOB->BSRRH = GPIO_BSRR_BR_10 >> 16;
	/* Should be high for external clock, do this after ADC in configured */

	/* Enable RCC for GPIOA for the external clock to the ADC on pin PA8 */
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; /* Enable clock for A pins */

	/* EOC is on PA9 and is an input pin by default */

	/* Configure the ADC digital input pins (all inputs by default) */
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; /* Enable clock for C pins */
}

/* Configure the ADC to use 4 analog channels */
static void configure_ADC_IC() {
	/* Temporarily make pins PC0 - PC7 output pins to configure ADC */
	GPIOC->MODER |=
		GPIO_MODER_MODER0_0 | GPIO_MODER_MODER1_0 | GPIO_MODER_MODER2_0 | GPIO_MODER_MODER3_0 |
		GPIO_MODER_MODER4_0 | GPIO_MODER_MODER5_0 | GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0;

	/* Drive WR low */
	GPIOB->BSRRH = GPIO_BSRR_BR_2 >> 16;

	/* Write configuration to GPIO C */
	GPIOC->ODR |= 0x0F;

	/* Delay a bit */
	delay_10ns(2); // 20 ns

	/* Drive WR high */
	GPIOB->BSRRL = GPIO_BSRR_BS_2;

	/* Delay a bit */
	delay_10ns(2); // 20 ns

	/* Return GPIOC to inputs */
	GPIOC->MODER &=
		~(GPIO_MODER_MODER0_0 | GPIO_MODER_MODER1_0 | GPIO_MODER_MODER2_0 | GPIO_MODER_MODER3_0 |
		  GPIO_MODER_MODER4_0 | GPIO_MODER_MODER5_0 | GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0);
}

/* External clock for ADC. PA8, TIM1_CH1 */
static void configure_clock_output() {
	/* Configure PA8 to Alternate Function TIM1_CH1 */
	GPIOA->MODER |= GPIO_MODER_MODER8_1; /* Alternate function mode */
	GPIOA->AFR[1] |= GPIO_AFRH_AFRH8_0; /* Alternate function 1 */

	/* Enable clock to the timer */
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

	/* Configure Tim1 Ch1 in PWM Mode 1 (High followed by low) */
	TIM1->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;

	/* Enable reload for channel 1 */
	TIM1->CCMR1 |= TIM_CCMR1_OC1PE;

	/* Enable capture/compare on channel 1 */
	TIM1->CCER |= TIM_CCER_CC1E;

	/* Enable outputs */
	TIM1->BDTR |= TIM_BDTR_MOE;

	/* Value to compare the current count with, dictates duty cycle */
	TIM1->CCR1 = EXT_CLOCK_DUTY;

	/* Counter reload value, dictates frequency */
	TIM1->ARR = EXT_CLOCK_PERIOD;

	/* Generate update event to load buffered values */
	TIM1->EGR |= TIM_EGR_UG;
	while(!(TIM1->SR & TIM_SR_UIF)); /* Wait for update event flag to raise */
	TIM1->SR &= ~TIM_SR_UIF; /* Clear the update event flag */

	/* Start clock */
	TIM1->CR1 |= TIM_CR1_ARPE | TIM_CR1_CEN;

	/* Enable external clock to ADC */
	GPIOB->BSRRL = GPIO_BSRR_BS_10;
}

static void configure_CONVST_PWM() {
	/* Configure CONVST on PB4 as alternate function mode with TIM3_CH1 */
	GPIOB->MODER |= GPIO_MODER_MODER4_1; /* Alternate function mode */
	GPIOB->AFR[0] |= GPIO_AFRL_AFRL4_1; /* Alternate function 2 */

	/* Enable clock to TIM3 */
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

	/* Configure Tim3 Ch1 in PWM Mode 1 (High followed by low) */
	TIM3->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;/* Enable reload for channel 1 */

	/* Enable reload for channel 1 */
	TIM3->CCMR1 |= TIM_CCMR1_OC1PE;

	/* Enable capture/compare on channel 1 */
	TIM3->CCER |= TIM_CCER_CC1E;

	/* Enable outputs */
	TIM3->BDTR |= TIM_BDTR_MOE;

	/* Value to compare the current count with, dictates duty cycle */
	TIM3->CCR1 = CONVST_DUTY;

	/* Counter reload value, dictates frequency */
	TIM3->ARR = CONVST_PERIOD;

	/* Generate update event to load buffered values */
	TIM3->EGR |= TIM_EGR_UG;
	while(!(TIM3->SR & TIM_SR_UIF)); /* Wait for update event flag to raise */
	TIM3->SR &= ~TIM_SR_UIF; /* Clear the update event flag */

	/* Start clock */
	TIM3->CR1 |= TIM_CR1_ARPE | TIM_CR1_CEN;
}

static void configure_RD_PWM() {

}

extern void init_ADC() {
	configure_ADC_GPIO();
	configure_ADC_IC();
	configure_clock_output();
	configure_CONVST_PWM();
	configure_RD_PWM();
}
