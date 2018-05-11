/*
 * PWM_OUT.c
 *
 *  Created on: Mar 27, 2018
 *      Author: Gabriel
 */
#include "stm32f4xx_gpio.h"

extern void init_PWM(){
	TIM_OCInitTypeDef OC_struct;

	OC_struct.TIM_OCMode = TIM_OCMode_PWM2;
	OC_struct.TIM_OutputState = TIM_OutputState_Enable;
	OC_struct.TIM_OCPolarity = TIM_OCPolarity_Low;

	/*pulse_length = ((TIM_Period + 1) * DutyCycle) / 100
	90% duty cycle: pulse_length = ((1200) * 80) / 100 = 960 */

	OC_struct.TIM_Pulse = 960 - 1; /* 90% duty cycle */
	TIM_OC1Init(TIM3, &OC_struct);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

}

extern void init_TIM3(){
	TIM_TimeBaseInitTypeDef Timer_struct;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	Timer_struct.TIM_Prescaler = 1 - 1;
	Timer_struct.TIM_CounterMode = TIM_CounterMode_Up;

	/*TIM_Period = timer_tick_frequency / PWM_frequency - 1 */

	Timer_struct.TIM_Period = 1200 - 1; //freq = 200kHz --assuming timer_tick = 84MHz
	Timer_struct.TIM_ClockDivision = TIM_CKD_DIV1;
	Timer_struct.TIM_RepetitionCounter = 0;

	TIM_TimeBaseInit(TIM3, &Timer_struct);
	TIM_Cmd(TIM3, ENABLE);
}

extern void init_GPIOB() {
    GPIO_InitTypeDef GPIO_InitStruct;

    /* Clock for GPIOB */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    /* Alternating functions for pins */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3);

    /* Set pins */
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
}

extern void init_GPIOB_LED(){
	GPIO_InitTypeDef GPIO_Outputs;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	//Pin 12 is LED, Pin0 & and Pin 2 are CS & WR
	GPIO_Outputs.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_2 | GPIO_Pin_12;
	GPIO_Outputs.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Outputs.GPIO_OType = GPIO_OType_PP;
	GPIO_Outputs.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Outputs.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_Outputs);
}
