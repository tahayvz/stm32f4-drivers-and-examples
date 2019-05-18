/*
 * stm32f407xx_ gpio_driver.h
 *
 *  Created on: 18 May 2019
 *      Author: tahay
 */

#ifndef INC_STM32F407XX__GPIO_DRIVER_H_
#define INC_STM32F407XX__GPIO_DRIVER_H_
#include "stm32f407xx.h"

typedef struct
{
	uint8_t GPIO_PinNumber;				/*possible values from @GPIO_PIN_NUMBERS*/
	uint8_t GPIO_PinMode; 				/*possible values from @GPIO_PIN_MODES*/
	uint8_t GPIO_PinSpeed;				/*possible values from @GPIO_PIN_SPEED*/
	uint8_t GPIO_PinPuPdControl;		/*possible values from @GPIO_PIN_PUPD*/
	uint8_t GPIO_PinOPType;				/*possible values from @GPIO_PIN_OPTYPE*/
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

/*
 * This is a Handle structure for a GPIO pin
 */

typedef struct
{
	GPIO_RegDef_t *pGPIOx;	//this holds the base address of the GPIO port to which the pin belongs
	GPIO_PinConfig_t GPIO_PinConfig; //this holds GPIO pin config settings
}GPIO__Handle_t;

/*
 *@GPIO_PIN_NUMBERS
 *GPIO pin numbers
 */
#define GPIO_PIN_NO_0		0
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2
#define GPIO_PIN_NO_3		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5		5
#define GPIO_PIN_NO_6		6
#define GPIO_PIN_NO_7		7
#define GPIO_PIN_NO_8		8
#define GPIO_PIN_NO_9		9
#define GPIO_PIN_NO_10		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15
/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */
#define GPIO_MODE_IN 		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALTFN 	2
#define GPIO_MODE_ANALOG 	3
#define GPIO_MODE_IT_FT		4	//gpio falling edge
#define GPIO_MODE_IT_RT		5	//gpio rising edge
#define GPIO_MODE_IT_RFT	6	//gpio mode rising edge falling edge

/*@GPIO_PIN_OPTYPE
 * GPIO pin possible output types
 */
#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OD		1

/*
 * @GPIO_PIN_SPEED
 * GPIO pin possible output speeds
 */

#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3

/*@GPIO_PIN_PUPD
 * GPIO pin pull up and pull down configuration macros
 */

#define GPIO_NO_PUPD		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2



/******************************************************************************************************
 *					APIs supported by this driver
 *		For more information about te APIs check the function definitions
 ******************************************************************************************************/

/*
 *Peripheral clock setup. GPIO enable disable
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * Init and De-Init port and pin
 */
void GPIO_Init(GPIO__Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 *Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WritetoOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WritetoOutputPort(GPIO_RegDef_t *pGPIOx, uint8_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQConfig(uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);



#endif /* INC_STM32F407XX__GPIO_DRIVER_H_ */
