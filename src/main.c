/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include <GPIO_Data.h>
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_gpio.h"
#include "misc.h"
#include <stdbool.h>
#include "ADC.h"
#include "PWM_OUT.h"
#include "READ_STATUS.h"

#include "FreeRTOSConfig.h"

#include "FreeRTOS.h"
#include "task.h"


uint16_t ch0_data = 0;
uint16_t ch1_data = 0;
uint16_t ch2_data = 0;
uint16_t ch3_data = 0;

int channel_read = 0;
int count = 0;

void blinkyTask(void *dummy){

	while(1){

		GPIOB->ODR ^= GPIO_Pin_12;
		vTaskDelay(500);
	}
}

void vGeneralTaskInit(void){
	xTaskCreate(blinkyTask,
		(const signed char *)"blinkyTask",
		configMINIMAL_STACK_SIZE,
		NULL,                 // pvParameters
		tskIDLE_PRIORITY + 1, // uxPriority
		NULL              ); // pvCreatedTask */

}

void temp_led_testing(void){
	//temp testing code for discovery
	GPIO_InitTypeDef GPIO_Outputs;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	//Pin 15 is blue LED
	GPIO_Outputs.GPIO_Pin = GPIO_Pin_15 | GPIO_Pin_12 | GPIO_Pin_14;
	GPIO_Outputs.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Outputs.GPIO_OType = GPIO_OType_PP;
	GPIO_Outputs.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Outputs.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_Outputs);

	GPIO_SetBits(GPIOD, GPIO_Pin_15); //test by led
}

int main(void)
{
	temp_led_testing();

	init_GPIOC(); //turn on all data pins
	init_GPIOB(); //GPIO for CONVST pwm
	init_TIM3(); //Timer for CONVST pwm
	init_PWM();

	init_GPIOB_LED(); //GPIO for LED pins & CS & WR signals

	/*Pull CS and WR low to allow write to configuration register*/
	GPIO_ResetBits(GPIOB, GPIO_Pin_0 | GPIO_Pin_2);
	/*write to configuration register*/
	GPIO_SetBits(GPIOC, GPIO_Pin_All & 0x00FF);

	//for(int x = 0; x <= 10000; x++ ){}	//delay not used when debugging
	for(int x = 0; x <= 3; x++ ){}	//temp

	/*Pull CS and WR high to disable write to configuration register*/
	GPIO_SetBits(GPIOB, GPIO_Pin_0 | GPIO_Pin_2);
	/*Reconfigure 14 Port C pins as input*/
	GPIOC->MODER = (GPIOC->MODER & ~(0x0FFFFFFF));

	vGeneralTaskInit();

	init_conv_ready(); //Interrupt setup at last possible moment

	vTaskStartScheduler();

	//Should never get here
	for(;;);
}

void vApplicationTickHook(void) {
}

/* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created.  It is also called by various parts of the
   demo application.  If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
void vApplicationMallocFailedHook(void) {
  taskDISABLE_INTERRUPTS();
  for(;;);
}

/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
   task.  It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()).  If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
void vApplicationIdleHook(void) {
}

void vApplicationStackOverflowHook(xTaskHandle pxTask, signed char *pcTaskName) {
  (void) pcTaskName;
  (void) pxTask;
  /* Run time stack overflow checking is performed if
     configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
     function is called if a stack overflow is detected. */
  taskDISABLE_INTERRUPTS();
  for(;;);
}

void EXTI9_5_IRQHandler(void) {

    if (EXTI_GetITStatus(EXTI_Line9) != RESET) {

    	GPIO_ToggleBits(GPIOB, GPIO_Pin_12);

    	if (channel_read == 0){
    		GPIO_ResetBits(GPIOB, GPIO_Pin_0 | GPIO_Pin_1); //pull RD down
    		ch0_data = (GPIOC->IDR & 0x3FFF);  				//read data
    		GPIO_SetBits(GPIOB, GPIO_Pin_1);   				//Set RD
    		channel_read++;
    	}
    	else if (channel_read == 1){
    		GPIO_ResetBits(GPIOB, GPIO_Pin_0 | GPIO_Pin_1); //pull CS & RD down
    		ch1_data = (GPIOC->IDR & 0x3FFF);  //read data
    		GPIO_SetBits(GPIOB, GPIO_Pin_1);   //Set RD
    		channel_read++;
    	}
    	else if (channel_read == 2){
    		GPIO_ResetBits(GPIOB, GPIO_Pin_0 | GPIO_Pin_1); //pull RD down
    	    ch2_data = (GPIOC->IDR & 0x3FFF);  //read data
    	    GPIO_SetBits(GPIOB, GPIO_Pin_1);   //Set RD
    	    channel_read++;
    	}
    	else if (channel_read == 3){
    		GPIO_ResetBits(GPIOB, GPIO_Pin_0 | GPIO_Pin_1); //pull RD down
    		ch3_data = (GPIOC->IDR & 0x3FFF);  //read data
    		GPIO_SetBits(GPIOB, GPIO_Pin_1);   //Set RD
    		channel_read = 0;
    	}
    	GPIO_SetBits(GPIOB, GPIO_Pin_0); //now also setting CS

    	//GPIO_SetBits(GPIOD, GPIO_Pin_14); /*Set LED on Discovery Board*/

    	/* Clear interrupt flag */
        EXTI_ClearITPendingBit(EXTI_Line0);
    }
}
