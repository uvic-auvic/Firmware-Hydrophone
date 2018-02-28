/*
 * LEDs.c
 *
 *  Created on: Aug 13, 2017
 *      Author: auvic
 */

#include "stm32f4xx_gpio.h"

extern void init_GPIOB(){
	GPIO_InitTypeDef GPIO_Outputs;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	GPIO_Outputs.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_Outputs.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Outputs.GPIO_OType = GPIO_OType_PP;
	GPIO_Outputs.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Outputs.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_Outputs);

	//GPIO_WriteBit(GPIOB, GPIO_Pin_12, Bit_SET);
	//GPIOB->BSRRL |= GPIO_Pin_14;
}

extern void init_GPIOA(){
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//GPIO_WriteBit(GPIOA, GPIO_Pin_12, Bit_SET);
	//GPIOA->BSRRL |= GPIO_Pin_14;
}
