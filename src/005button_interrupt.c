/*
 * 005button_interrupt.c
 *
 *  Created on: 19 May 2019
 *      Author: tahay
 */


#include <string.h> 		//included for memset
#include "stm32f407xx.h"
#define HIGH 1
#define LOW  0
#define BTN_PRESSED LOW

void delay()
{
	//this will introduce ~200ms delay when system clock is 16MHz
	for(uint32_t i=0; i<500000/2;i++);
}

int main(void){

	GPIO__Handle_t GpioLed,GPIOBtn;
	//used memset because  GPIO_Init(&GPIOBtn); function have wrong bits. We did not check bits befor
	memset(&GpioLed,0,sizeof(GpioLed));//set each member of this element of this structure to 0
	memset(&GPIOBtn,0,sizeof(GpioLed));
	GpioLed.pGPIOx=GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_LOW;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;


	GPIO_PeriClockControl(GPIOD,ENABLE);
	GPIO_Init(&GpioLed);

	GPIOBtn.pGPIOx=GPIOD;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_5;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IT_FT;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PIN_PU;
	GPIO_PeriClockControl(GPIOA,ENABLE);

	GPIO_Init(&GPIOBtn);

	//IRQ Configuration. Priority config defined when used multiple interrupt
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5,ENABLE);

	while(1);
	return 0;
}
void EXTI9_5_IRQHandler(void) //checked name in startup_stm32.s
{
	delay();//200ms. Wait till button de-bouncing gets over
	GPIO_IRQHandling(GPIO_PIN_NO_5);
	GPIO_ToggleOutputPin(GPIOD,GPIO_PIN_NO_12);
}

