#include "stm32f4xx_usart.h"
#include "DMA.h"

volatile uint16_t ADC_values[ARRAYSIZE] = {1, 2, 3, 4, 5, 6, 7, 8};
volatile uint16_t Buffer[ARRAYSIZE + 2];
volatile uint16_t fft[ARRAYSIZE + 2];
volatile uint8_t dma_state = 0;

static void Configure_DMA2_Stream2(void){
	//enable DMA2 clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

	//reset DMA2 stream 1 to default values;
	DMA_DeInit(DMA2_Stream2);

	// Select stream 2 channel 6, in circular mode, with high priority,
	// peripheral-to-memory, and enable transfer complete flag
	DMA2_Stream2->CR |= DMA_SxCR_CHSEL_2 | DMA_SxCR_CHSEL_1 | DMA_SxCR_PL_1 | DMA_SxCR_PL_0| DMA_SxCR_CIRC;
	DMA2_Stream2->CR |= DMA_SxCR_TCIE;

	// Configure both source and destination to half-word (2 bytes)
	DMA2_Stream2->CR |= DMA_SxCR_MSIZE_1 | DMA_SxCR_PSIZE_1;

	// Enable auto-increment of source and destination pointers
	DMA2_Stream2->CR |= DMA_SxCR_MINC | DMA_SxCR_PINC;

	// Configure the number of bytes to send per transfer
	DMA2_Stream2->NDTR = ARRAYSIZE;

	// Select the source address for memory-to-memory mode
	DMA2_Stream2->PAR = (uint32_t)ADC_values;

	// Select the destination address for memory-to-memory mode
	DMA2_Stream2->M0AR = (uint32_t)Buffer;

	// Enable DMA2
	DMA2_Stream2->CR |= DMA_SxCR_EN;
}

static void Configure_DMA2_Stream1(void){
	//enable DMA2 clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

	//reset DMA2 stream 1 to default values;
	DMA_DeInit(DMA2_Stream1);

	// Select sream 1 channel 6, in circular mode, with high priority,
	// peripheral-to-memory, and enable transfer complete flag
	DMA2_Stream1->CR |= DMA_SxCR_CHSEL_2 | DMA_SxCR_CHSEL_1 | DMA_SxCR_PL_1 | DMA_SxCR_PL_0| DMA_SxCR_CIRC;
	DMA2_Stream1->CR |= DMA_SxCR_TCIE;

	// Configure both source and destination to half-word (2 bytes)
	DMA2_Stream1->CR |= DMA_SxCR_MSIZE_1 | DMA_SxCR_PSIZE_1;

	// Enable auto-increment of source and destination pointers
	DMA2_Stream1->CR |= DMA_SxCR_MINC | DMA_SxCR_PINC;

	// Configure the number of bytes to send per transfer
	DMA2_Stream1->NDTR = ARRAYSIZE;

	// Select the source address for memory-to-memory mode
	DMA2_Stream1->PAR = (uint32_t)ADC_values;

	// Select the destination address for memory-to-memory mode
	DMA2_Stream1->M0AR = (uint32_t)Buffer;

	// Enable DMA2
	DMA2_Stream1->CR |= DMA_SxCR_EN;
}

static void Configure_DMA1(void) {
	//enable DMA1 clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

	//reset DMA1 channel to default values;
	DMA_DeInit(DMA1_Stream6); //stream 6 for usart2 tx

	// Select stream6 channel 4, in double-buffer mode, high priority,
	// memory-to-peripheral, and enable transfer-complete interrupt
	DMA1_Stream6->CR |= DMA_SxCR_CHSEL_2 | DMA_SxCR_DBM | DMA_SxCR_PL_1 | DMA_SxCR_DIR_0 | DMA_SxCR_TCIE;

	// Configure both source and destination to half-word (2 bytes)
	DMA1_Stream6->CR |= DMA_SxCR_MSIZE_1 | DMA_SxCR_PSIZE_1;

	// Enable auto-increment of source and destination pointers
	DMA1_Stream6->CR |= DMA_SxCR_MINC | DMA_SxCR_PINC;

	// Specify the number of half-words to use per transaction
	DMA1_Stream6->NDTR = ARRAYSIZE;

	// Select the destination address for the USART2 TX peripheral
	DMA1_Stream6->PAR = (uint32_t)&(USART2->DR);

	// Select the source address for the raw data
	DMA1_Stream6->M0AR = (uint32_t)Buffer;

	// Select the source address for the fft data
	DMA1_Stream6->M1AR = (uint32_t)fft;
}

static void Configure_NVIC_DMA(void){
	// Enable IRQ for grabbing data from ADC pins
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream1_IRQn; // TIM1_CH1
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// Enable IRQ for grabbing data from ADC pins
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream2_IRQn; // TIM1_CH2
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// Enable IRQ for sending data to USART2 TX
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

extern void DMA_init(void){
	Configure_DMA2_Stream2();
	//Configure_DMA2_Stream1();
	//Configure_DMA1();
	Configure_NVIC_DMA();
}

void DMA2_Stream1_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA2_Stream1, DMA_IT_TCIF1)){
		int i = 0;
		for(i = 0; i < ARRAYSIZE; i++){
			ADC_values[i]++;
		}
/*
		// Change to state 1
		dma_state = 1;

		// Disable transfer on this DMA controller
		DMA2_Stream1->CR &= ~DMA_SxCR_EN;

		// Select mem0 for double-buffer, and then enable DMA1
		DMA1_Stream6->CR &= ~DMA_SxCR_CT;
		DMA1_Stream6->CR |= DMA_SxCR_EN;*/

		DMA_ClearITPendingBit(DMA2_Stream1, DMA_IT_TCIF1);
		DMA_ClearFlag(DMA2_Stream1, DMA_IT_TCIF1);
	}
}

void DMA2_Stream2_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA2_Stream2, DMA_IT_TCIF2)){
		int i = 0;
		for(i = 0; i < ARRAYSIZE; i++){
			ADC_values[i]++;
		}
/*
		// Change to state 1
		dma_state = 1;

		// Disable transfer on this DMA controller
		DMA2_Stream1->CR &= ~DMA_SxCR_EN;

		// Select mem0 for double-buffer, and then enable DMA1
		DMA1_Stream6->CR &= ~DMA_SxCR_CT;
		DMA1_Stream6->CR |= DMA_SxCR_EN;*/

		DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_TCIF2);
		DMA_ClearFlag(DMA2_Stream2, DMA_IT_TCIF2);
	}
}

void DMA1_Stream6_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA2_Stream6, DMA_IT_TCIF4)){
		// Disable DMA1
		DMA1_Stream6->CR &= ~DMA_SxCR_EN;

		switch(dma_state){
		case 1:
			dma_state = 2;
			break;
		case 2:
		default:
			dma_state = 0;
		}

		DMA_ClearITPendingBit(DMA2_Stream6, DMA_IT_TCIF4);
		DMA_ClearFlag(DMA2_Stream6, DMA_IT_TCIF4);
	}
}

///*wait for DMA transfer to be done*/
//while(dma2_status == 0) {}
//
//for(int i = 0, i < ARRAYSIZE, i++)
//{
//	//destination[i]=source[i];
//}
