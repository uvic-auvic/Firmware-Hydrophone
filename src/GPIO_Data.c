#include "stm32f4xx_gpio.h"


extern void init_GPIOC(){

	GPIO_InitTypeDef GPIO_Ports;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	GPIO_StructInit(&GPIO_Ports);
	GPIO_Ports.GPIO_Pin = GPIO_Pin_All & 0x3FFF;
	GPIO_Ports.GPIO_Mode = GPIO_Mode_IN;
	GPIO_Ports.GPIO_OType = GPIO_OType_PP;
	GPIO_Ports.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Ports.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_Ports);
}

//extern void init_GPIOA(){
//	GPIO_InitTypeDef GPIO_InitStructure;
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
//
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_Init(GPIOB, &GPIO_InitStructure);
//
//
//}
