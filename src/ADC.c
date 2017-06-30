#include "stm32f4xx_adc.h"
#include "ADC.h"

static void Configure_ADC(void){
	/*enable peripheral clock for ADC1 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	ADC_InitTypeDef ADC_InitStructure;
	//ADC1 configuration

	//select continuous conversion mode
	//ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;

	//We will convert multiple channels
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	//we will convert one time
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	//select no external triggering
	ADC_InitStructure.ADC_ExternalTrigConvEdge = 0;
	ADC_InitStructure.ADC_ExternalTrigConv = 0;
	//right 12-bit data alignment in ADC data register
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	//8 ADC conversions for sequencer
	ADC_InitStructure.ADC_NbrOfConversion = 8;

	//load structure values to control and status registers
	ADC_Init(ADC1, &ADC_InitStructure);

	//channel configuration
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_56Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_56Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_56Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_56Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 5, ADC_SampleTime_56Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 6, ADC_SampleTime_56Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 7, ADC_SampleTime_56Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 8, ADC_SampleTime_56Cycles);

	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2; //appropriate speed?
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_8Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);

	//Enables or disables the ADC DMA request after last transfer (Single-ADC mode)
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
	//Enable ADC1

	ADC_Cmd(ADC1, ENABLE);
	//enable DMA for ADC
	ADC_DMACmd(ADC1, ENABLE);
}

static void Configure_GPIO_ADC(void){
	/**
	* Configuring GPIOA for ADC
	*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure; //Variable used to setup the GPIO pins
	//==Configure ADC pins (PA0 -> Channel 0 to PA7 -> Channel 7) as analog inputs==
	GPIO_StructInit(&GPIO_InitStructure); // Reset init structure, if not it can cause issues...
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1| GPIO_Pin_2| GPIO_Pin_3| GPIO_Pin_4| GPIO_Pin_5| GPIO_Pin_6| GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

extern void ADC_init(void){
	Configure_GPIO_ADC();
	Configure_ADC();
}
