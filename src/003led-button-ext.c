/*
 * 003led-button-ext.c
 *
 *  Created on: 19 May 2019
 *      Author: tahay
 */

#include "stm32f407xx.h"
#define HIGH 1
#define LOW  0
#define BTN_PRESSED LOW

void delay(){
	for(uint32_t i=0; i<500000/2;i++);
}

int main(void){

	GPIO__Handle_t GpioLed,GPIOBtn;

	GpioLed.pGPIOx=GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_8;
	GpioLed.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;


	GPIO_PeriClockControl(GPIOB,ENABLE);
	GPIO_Init(&GpioLed);

	GPIOBtn.pGPIOx=GPIOB;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_12;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PIN_PU; //there is already pull down resistor in button pin

	GPIO_PeriClockControl(GPIOA,ENABLE);

	GPIO_Init(&GPIOBtn);

	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOB,GPIO_PIN_NO_12)==BTN_PRESSED) //pressed button return 1
		{
			delay(); //clearlt see toggle
			GPIO_ToggleOutputPin(GPIOD,GPIO_PIN_NO_8);
		}
	}
	return 0;
}


