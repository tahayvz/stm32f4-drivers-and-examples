/*
 * 001led_toggle.c
 *
 *  Created on: 18 May 2019
 *      Author: tahay
 */

#include "stm32f407xx.h"

void delay(){
	for(uint32_t i=0; i<500000;i++);
}
int main(void){

	GPIO__Handle_t GpioLed;

	GpioLed.pGPIOx=GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_OD;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PIN_PU;
	//used external resistor. PD12 connected with wire to Vcc 5 on STM 
	GPIO_PeriClockControl(GPIOD,ENABLE);
	GPIO_Init(&GpioLed); //before configuring this peripheral we enabled peripheral control. this function going lots of peripheral initialization

	while(1)
	{
		GPIO_ToggleOutputPin(GPIOD,GPIO_PIN_NO_12);
		delay();
	}
	return 0;
} 
