/*
 * ADC.c
 *
 *  Created on: Jun 1, 2018
 *      Author: Lyden
 */

#include "stm32f4xx_rcc.h"
#include "delay.h"

#define GPIO_AFRH_AFRH9_0 (0x00000010)
#define GPIO_AFRH_AFRH9_1 (0x00000020)
#define GPIO_AFRH_AFRH9_2 (0x00000040)
#define GPIO_AFRH_AFRH9_3 (0x00000080)

#define GPIO_AFRH_AFRH8_0 (0x00000001)
#define GPIO_AFRH_AFRH8_1 (0x00000002)
#define GPIO_AFRH_AFRH8_2 (0x00000004)
#define GPIO_AFRH_AFRH8_3 (0x00000008)

#define GPIO_AFRL_AFRL4_0 (0x00010000)
#define GPIO_AFRL_AFRL4_1 (0x00020000)
#define GPIO_AFRL_AFRL4_2 (0x00040000)
#define GPIO_AFRL_AFRL4_3 (0x00080000)

#define GPIO_AFRL_AFRL1_0 (0x00000010)
#define GPIO_AFRL_AFRL1_1 (0x00000020)
#define GPIO_AFRL_AFRL1_2 (0x00000040)
#define GPIO_AFRL_AFRL1_3 (0x00000080)

/* 100MHz / 5 = 20MHz */
//#define ADC_TIMER_PRESCALAR (5 - 1)

/* 5 Clock Cycles at 20MHz produces a 4MHz signal */
#define EXT_CLOCK_PERIOD (5)

/* CONVST PWM with computed period and a on time of 37 cycles */
#define CONVERSION_TIME (37)
#define ACQUISITION_TIME (10)
//#define CONVST_PERIOD (((CONVERSION_TIME + ACQUISITION_TIME) * EXT_CLOCK_PERIOD) - 1)
//#define CONVST_DUTY (CONVERSION_TIME * EXT_CLOCK_PERIOD)

/* RD PWM with 50% duty, 3.33 MHz frequency */
//#define RD_PERIOD (0xFFFF)
//#define RD_DUTY (EXT_CLOCK_PERIOD + 1)
//#define RD_PSV (0)

// Defines below are for testing purposes (I can't see really fast things!)
#define RD_PERIOD 	(0xFFFF)
#define RD_DUTY   	(10000)
#define RD_PSC		(0x1000)
#define ADC_TIMER_PRESCALAR (0xFFFF)
#define CONVST_PERIOD (2000)
#define CONVST_DUTY (1000)

#define ADC_MAX_READINGS (1024)
#define ADC_CHANNEL_COUNT (4)

uint16_t ADC_Buffer[ADC_MAX_READINGS * ADC_CHANNEL_COUNT];
uint16_t buffer_size = ADC_MAX_READINGS * ADC_CHANNEL_COUNT;

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
	/* MCO1 uses HSI (16 MHz) with a prescalar division of 4 to produce 4MHz */
	RCC->CFGR |= RCC_CFGR_MCO1PRE_2 | RCC_CFGR_MCO1PRE_1;

	/* Configure PA8 to Alternate Function 0 MCO_1 */
	GPIOA->MODER |= GPIO_MODER_MODER8_1; /* Alternate function mode */

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
	TIM3->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;

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

	/* Get the prescalar */
	TIM3->PSC = ADC_TIMER_PRESCALAR;

	/* Generate update event to load buffered values */
	TIM3->EGR |= TIM_EGR_UG;
	while(!(TIM3->SR & TIM_SR_UIF)); /* Wait for update event flag to raise */
	TIM3->SR &= ~TIM_SR_UIF; /* Clear the update event flag */

	/* Start clock */
	TIM3->CR1 |= TIM_CR1_ARPE | TIM_CR1_CEN;
}

static void configure_RD_EOC_TIM() {
	/* Configure RD (PB1) in alternate function mode for TIM1 CH3N */
	GPIOB->MODER &= ~GPIO_MODER_MODER1_0; /* Get it out of output mode */
	GPIOB->MODER |= GPIO_MODER_MODER1_1; /* Put it in alternate function mode */
	GPIOB->AFR[0] |= GPIO_AFRL_AFRL1_0; /* Alternate function 1 */

	/* Configure EOC (PA9) in alternate function mode for TIM1 CH2 */
	GPIOA->MODER |= GPIO_MODER_MODER9_1; /* Put it in alternate function mode */
	GPIOA->AFR[1] |= GPIO_AFRH_AFRH9_0; /* Alternate function 1 */

	// TEST - PE11 alternate functoin mode for TIM1 CH2
	/*RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
	GPIOE->MODER |= GPIO_MODER_MODER11_1;
	GPIOE->AFR[1] |= 0x00001000;
*/
	/* Enable the clock to TIM1 */
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

	/***** Setup PWM for RD pin (PB1) *****/

	/* Setup frequency and duty cycle of PWM */
	TIM1->CCR3 = RD_DUTY;
	TIM1->ARR = RD_PERIOD;
	TIM1->PSC = RD_PSC;

	/* Choose PWM mode 1 NOT(High in beginning, low afterwards)	*/
	TIM1->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_0;

	/* Enable reload for channel 1 */
	TIM1->CCMR2 |= TIM_CCMR2_OC3PE;

	/* Enable capture/compare on channel 1 */
	TIM1->CCER |= TIM_CCER_CC3NE;

	/* Enable outputs */
	TIM1->BDTR |= TIM_BDTR_MOE;

	/***** Setup input mode for EOC pin (PA9) *****/

	/* Map channel 2 trigger input to capture/compare channel 1 */
	TIM1->CCMR1 |= TIM_CCMR1_CC2S_0;

	/* Configure for falling edge */
	TIM1->CCER |= TIM_CCER_CC2P;

	/* Put timer in reset on Internal Trigger 2 edge */
	TIM1->SMCR |= TIM_SMCR_SMS_2 | TIM_SMCR_TS_1 | TIM_SMCR_TS_2;

	/* Enable DMA Request when Trigger detected */
	TIM1->DIER |= TIM_DIER_TDE;

	/* Generate update event to load buffered values */
	TIM1->EGR |= TIM_EGR_UG;
	while(!(TIM1->SR & TIM_SR_UIF)); /* Wait for update event flag to raise */
	TIM1->SR &= ~TIM_SR_UIF; /* Clear the update event flag */

	/* Start clock */
	TIM1->CR1 |= TIM_CR1_ARPE | TIM_CR1_CEN;
}

static void configure_EOC_DMA() {
	/* Enable clock to DMA2 */
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

	/* Make sure DMA stream is off */
	DMA2_Stream4->CR &= ~DMA_SxCR_EN;
	while((DMA2_Stream4->CR & DMA_SxCR_EN)); /* Wait until stream is disabled */

	/* Set the peripheral address */
	DMA2_Stream4->PAR = (uint32_t)(&(GPIOC->IDR));

	/* Set the memory address */
	DMA2_Stream4->M0AR = (uint32_t)ADC_Buffer;

	/* Set the number of data items to be transferred */
	DMA2_Stream4->NDTR = buffer_size;

	/* Select DMA Channel 6 */
	DMA2_Stream4->CR |= DMA_SxCR_CHSEL_2 | DMA_SxCR_CHSEL_1;

	/* Configure Priority level to very high */
	DMA2_Stream4->CR |= DMA_SxCR_PL_1 | DMA_SxCR_PL_0;

	/* Configure sizes to 16 bits, and memory auto-increment */
	DMA2_Stream4->CR |= DMA_SxCR_MSIZE_0 | DMA_SxCR_PSIZE_0 | DMA_SxCR_MINC;

	/* Enable DMA!!!! */
	DMA2_Stream4->CR |= DMA_SxCR_EN;

	for(int i = 0; i < 10000; ++i);
}

extern void init_ADC() {
	configure_ADC_GPIO();
	configure_ADC_IC();
	configure_clock_output();
	configure_CONVST_PWM();
	configure_RD_EOC_TIM();
	configure_EOC_DMA();
}
