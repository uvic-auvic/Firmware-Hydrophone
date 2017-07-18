#include "stm32f4xx.h"
#include "Timer.h"

static void Configure_TIM1_CH1(void){
	/* TIM1 clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	// Configure channel 1 as input to TI1
	TIM1->CCMR1 |= TIM_CCMR1_CC1S_0;
	// Apply filter with N=8, and trigger on filtered TI1
	TIM1->SMCR |= TIM_SMCR_ETF_0 | TIM_SMCR_ETF_1 | TIM_SMCR_TS_2 | TIM_SMCR_TS_0;
	// Enable DMA request on chanel 1
	TIM1->DIER |= TIM_DIER_CC1DE;
	// Enable capture, with non-inverting rising edge trigger
	TIM1->CCER |= TIM_CCER_CC1E;
}

static void Configure_TIM1_CH2(void){
	/* TIM1 clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	// Configure channel 2 as input to TI2
	TIM1->CCMR1 |= TIM_CCMR1_CC2S_0;
	// Apply filter with N=8, and trigger on filtered TI2
	TIM1->SMCR |= TIM_SMCR_ETF_0 | TIM_SMCR_ETF_1 | TIM_SMCR_TS_2 | TIM_SMCR_TS_1;
	// Enable DMA request on chanel 2
	TIM1->DIER |= TIM_DIER_CC2DE;
	// Enable capture, with non-inverting rising edge trigger
	TIM1->CCER |= TIM_CCER_CC2E;
}


static void Configure_GPIO_TIM1_CH1(void){
	GPIO_InitTypeDef GPIO_InitStructure;

	/* GPIOA clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	/* GPIOA Configuration: TIM1 CH1 (PA8) */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; // Input/Output controlled by peripheral
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Connect TIM1 pins to AF */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1); // PA8 - TIM1_CH1
}

static void Configure_GPIO_TIM1_CH2(void){
	GPIO_InitTypeDef GPIO_InitStructure;

	/* GPIOA clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	/* GPIOA Configuration: TIM1 CH2 (PA9) */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; // Input/Output controlled by peripheral
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Connect TIM1 pins to AF */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_TIM1); // PA9 - TIM1_CH2
}

extern void timer_init(void){
	//Configure_GPIO_TIM1_CH1();
	Configure_GPIO_TIM1_CH2();
	//Configure_TIM1_CH1();
	Configure_TIM1_CH2();
}
