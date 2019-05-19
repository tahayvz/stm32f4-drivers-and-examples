/*
 * main.c
 *
 *  Created on: 19 May 2019
 *      Author: tahay
 */

#include "stm32f407xx.h"

int main(void)
{

	return 0;
}

void EXTI0_IRQHandler(void)
{
	//handler the interrupt
	GPIO_IRQHandling(0);//0 is pin number

}
