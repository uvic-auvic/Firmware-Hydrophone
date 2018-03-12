
#include "stm32f4xx_gpio.h"

extern void init_PWM(){
	TIM_OCInitTypeDef OC_struct;

	OC_struct.TIM_OCMode = TIM_OCMode_PWM2;
	OC_struct.TIM_OutputState = TIM_OutputState_Enable;
	OC_struct.TIM_OCPolarity = TIM_OCPolarity_Low;

	/*pulse_length = ((TIM_Period + 1) * DutyCycle) / 100 - 1
	25% duty cycle: pulse_length = ((8399 + 1) * 25) / 100 - 1 = 2099 */

	OC_struct.TIM_Pulse = 2100 - 1; /* 25% duty cycle */
	TIM_OC1Init(TIM4, &OC_struct);
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

}

extern void init_TIM4(){
	TIM_TimeBaseInitTypeDef Timer_struct;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	Timer_struct.TIM_Prescaler = 1 - 1;
	Timer_struct.TIM_CounterMode = TIM_CounterMode_Up;

	/*TIM_Period = timer_tick_frequency / PWM_frequency - 1 */

	Timer_struct.TIM_Period = 8400 - 1; //freq = 10kHz assuming timer_tick = 84MHz
	Timer_struct.TIM_ClockDivision = TIM_CKD_DIV1;
	Timer_struct.TIM_RepetitionCounter = 0;

	TIM_TimeBaseInit(TIM4, &Timer_struct);
}
extern void TM_LEDS_Init() {
    GPIO_InitTypeDef GPIO_InitStruct;

    /* Clock for GPIOD */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    /* Alternating functions for pins */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM4);

    /* Set pins */
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
}

extern void init_GPIOC(){
	GPIO_InitTypeDef GPIO_Ports;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	GPIO_StructInit(&GPIO_Ports);
	GPIO_Ports.GPIO_Pin = GPIO_Pin_All & 0x3FFF;
	GPIO_Ports.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Ports.GPIO_OType = GPIO_OType_PP;
	GPIO_Ports.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Ports.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_Ports);
}

//extern void init_GPIOB(){
//	GPIO_InitTypeDef GPIO_Outputs;
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
//
//	GPIO_Outputs.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
//	GPIO_Outputs.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_Outputs.GPIO_OType = GPIO_OType_PP;
//	GPIO_Outputs.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Outputs.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_Init(GPIOB, &GPIO_Outputs);
//}

extern void init_GPIOA(){
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);


}
