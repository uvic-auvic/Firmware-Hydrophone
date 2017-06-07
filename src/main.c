/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/

#include "stm32f4xx.h"
//#include "usart.h"
#include "main.h"
#define ARRAYSIZE 8*4
#define ADC1_DR    ((uint32_t)0x4001244C)
volatile uint16_t ADC_values[ARRAYSIZE];
volatile uint32_t status = 0;
			



void ADCInit(void);
void DMAInit(void);
int main(void)
{
uint8_t index;
//initialize USART1
//Usart1Init();
ADCInit();
DMAInit();

//Enable DMA2 Channel transfer
DMA_Cmd(DMA2_Channel1, ENABLE);
//Start ADC1 Software Conversion
ADC_SoftwareStartConvCmd(ADC1, ENABLE);
//wait for DMA complete
while (!status){};
ADC_SoftwareStartConvCmd(ADC1, DISABLE);
//print averages
/*for(index = 0; index<8; index++)
	{
	printf("ch%d = %d ",index, ADC_values[index]);
	}*/
for(index = 0; index<8; index++){
	printf("\r\n ADC value on ch%d = %d\r\n",
			index, (uint16_t)((ADC_values[index]+ADC_values[index+8]
					+ADC_values[index+16]+ADC_values[index+24])/4));
}

while (1)
  {
	//interrupts does the job
  }
}

void ADCInit(void){
	//--Enable ADC1 and GPIOA--
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure; //Variable used to setup the GPIO pins
	//==Configure ADC pins (PA0 -> Channel 0 to PA7 -> Channel 7) as analog inputs==
	GPIO_StructInit(&GPIO_InitStructure); // Reset init structure, if not it can cause issues...
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1| GPIO_Pin_2| GPIO_Pin_3| GPIO_Pin_4| GPIO_Pin_5| GPIO_Pin_6| GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	ADC_InitTypeDef ADC_InitStructure;
	//ADC1 configuration
	//select continuous conversion mode
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	//We will convert multiple channels
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	//we will convert one time
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;//!
	//select no external triggering
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	//right 12-bit data alignment in ADC data register
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	//8 channels conversion
	ADC_InitStructure.ADC_NbrOfChannel = 8;
	//load structure values to control and status registers
	ADC_Init(ADC1, &ADC_InitStructure);
	//wake up temperature sensor
	//ADC_TempSensorVrefintCmd(ENABLE);
	//configure each channel
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_41Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_41Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_41Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_41Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 5, ADC_SampleTime_41Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 6, ADC_SampleTime_41Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 7, ADC_SampleTime_41Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 8, ADC_SampleTime_41Cycles5);
	//Enable ADC1
	ADC_Cmd(ADC1, ENABLE);
	//enable DMA for ADC
	ADC_DMACmd(ADC1, ENABLE);
	//Enable ADC1 reset calibration register
	ADC_ResetCalibration(ADC1);
	//Check the end of ADC1 reset calibration register
	while(ADC_GetResetCalibrationStatus(ADC1));
	//Start ADC1 calibration
	ADC_StartCalibration(ADC1);
	//Check the end of ADC1 calibration
	while(ADC_GetCalibrationStatus(ADC1));
}
void DMAInit(void){
	//enable DMA2 clock
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);
	//create DMA structure
	DMA_InitTypeDef  DMA_InitStructure;
	//reset DMA2 channe1 to default values;
	DMA_DeInit(DMA2_Channel1);
	//channel will be used for memory to memory transfer
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	//setting normal mode (non circular)
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	//medium priority
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	//source and destination data size word=32bit
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	//automatic memory destination increment enable.
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	//source address increment disable
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	//Location assigned to peripheral register will be source
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	//chunk of data to be transfered
	DMA_InitStructure.DMA_BufferSize = ARRAYSIZE;
	//source and destination start addresses
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADC_values;
	//send values to DMA registers
	DMA_Init(DMA2_Channel1, &DMA_InitStructure);
	// Enable DMA2 Channel Transfer Complete interrupt
	DMA_ITConfig(DMA2_Channel1, DMA_IT_TC, ENABLE);
	DMA_Cmd(DMA2_Channel1, ENABLE); //Enable the DMA2 - Channel1
	NVIC_InitTypeDef NVIC_InitStructure;
	//Enable DMA2 channel IRQ Channel */
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Channel1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);


}
