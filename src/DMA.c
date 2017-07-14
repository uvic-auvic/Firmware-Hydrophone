#include "stm32f4xx_usart.h"
#include "DMA.h"

#define ARRAYSIZE 8

volatile uint16_t ADC_values[ARRAYSIZE];
volatile uint16_t ADC_values2[ARRAYSIZE];
int dma2_status = 0;

static void Configure_DMA1(void){
	//enable DMA2 clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	//create DMA structure
	DMA_InitTypeDef  DMA_InitStructure;
	//reset DMA2 channe1 to default values;
	DMA_DeInit(DMA2_Stream0);

	DMA_InitStructure.DMA_Channel = DMA_Channel_0;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;  //not using FIFO
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull; //not using FIFO
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

	//setting circular mode
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
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	//chunk of data to be transfered
	DMA_InitStructure.DMA_BufferSize = ARRAYSIZE;
	//source and destination start addresses
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC_values[0];

	//CONFIGURE DOUBLE BUFFER MODE, MEMORY0BASEADDR configured as current address
	DMA_DoubleBufferModeConfig(DMA2_Stream0,&ADC_values2, DMA_Memory_0);
	DMA_DoubleBufferModeCmd(DMA2_Stream0, ENABLE);

	DMA_Init(DMA2_Stream0, &DMA_InitStructure);
	// Enable DMA2 Channel Transfer Complete interrupt
	DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, ENABLE);
	DMA_Cmd(DMA2_Stream0, ENABLE); //Enable the DMA2 - Stream 0
}

static void Configure_DMA2(void) {
	//enable DMA2 clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	//create DMA structure
	DMA_InitTypeDef  DMA_InitStructure;
	//reset DMA2 channel to default values;
	DMA_DeInit(DMA1_Stream6); //stream 6 for usart2 tx

	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

	//setting circular mode
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	//medium priority
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	//source and destination data size word=32bit
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	//automatic memory destination increment enable.
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	//source address increment disable
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	//Location assigned to peripheral register will be source
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral ;
	//chunk of data to be transfered
	DMA_InitStructure.DMA_BufferSize = ARRAYSIZE;
	//source and destination start addresses
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC_values[0]; //needs a buffer
	//send values to DMA registers
	DMA_Init(DMA1_Stream6, &DMA_InitStructure);
	// Enable DMA2 Channel Transfer Complete interrupt
	DMA_ITConfig(DMA1_Stream6, DMA_IT_TC, ENABLE);
	DMA_Cmd(DMA1_Stream6, ENABLE);
}

static void Configure_NVIC_DMA(void){
	//Enable DMA2 channel IRQ Channel */
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

extern void DMA_init(void){
	Configure_DMA2();
	Configure_DMA1();
	Configure_NVIC_DMA();
}

void DMA2_Stream0_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA2_Stream0, DMA_IT_HTIF0))
	{
		dma2_status = 2;
	}
	if(DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0))
	{
		dma2_status = 1;
	}
	DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0 | DMA_IT_HTIF0);
	DMA_ClearFlag(DMA2_Stream0, DMA_IT_TCIF0 | DMA_IT_HTIF0);
}

///*wait for DMA transfer to be done*/
//while(dma2_status == 0) {}
//
//for(int i = 0, i < ARRAYSIZE, i++)
//{
//	//destination[i]=source[i];
//}
