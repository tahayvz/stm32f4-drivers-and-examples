/*
 * stm32f407xx.h
 *
 *  Created on: 17 May 2019
 *      Author: tahay
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

#define __vo volatile

/*********************START:Processor Specific Details******************************
 *
 * ARM Cortex Processor NVIC ISERx register addresses
 */

#define NVIC_ISER0			((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1			((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2			((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3			((__vo uint32_t*)0xE000E10C)

 /*
 * ARM Cortex Processor NVIC ICERx register addresses
 */

#define NVIC_ICER0			((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1			((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2			((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3			((__vo uint32_t*)0xE000E18C)

 /*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */

#define NVIC_PR_BASE_ADDR 	((__vo uint32_t*)0xE000E400)

 /*
 * ARM Cortex Mx Processor number of priority bits implemented in priority register
 */

#define NO_PR_BITS_IMPLEMENTED			4

/*
 * base address of Flash and SRAM memories
 */
#define FLASB_BASEADDR						0x08000000U //U mean unsigned int
#define SRAM1_BASEADDR						0x20000000U //112kb
#define SRAM2_BASEADDR					 	0x20001C00U
#define ROM_BASEADDR						0x1FFF0000U
#define SRAM								SRAM1_BASEADDR //instead of "_BASE" use "_BASEADDR" as above macro
/*
 * AHBx and APB Bus peripheral base addresses
 */
#define PERIPH_BASE							0X40000000U
#define APB1PERIPH_BASEADDR						PERIPH_BASE
#define APB2PERIPH_BASEADDR						0X40010000U
#define AHB1PERIPH_BASEADDR						0X40020000U
#define AHB2PERIPH_BASEADDR						0X50000000U

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 */

#define GPIOA_BASEADDR						(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR						(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR						(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR						(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR						(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR						(AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR						(AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR						(AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR						(AHB1PERIPH_BASEADDR + 0x2000)
#define RCC_BASEADDR						(AHB1PERIPH_BASEADDR + 0X3800)
/*
 * Base address of peripherals which are hanging on APB1 bus
 */
#define I2C1_BASEADDR						(APB1PERIPH_BASEADDR+0X5400)
#define I2C2_BASEADDR						(APB1PERIPH_BASEADDR+0X5800)
#define I2C3_BASEADDR						(APB1PERIPH_BASEADDR+0X5C00)

#define SPI2_BASEADDR						(APB1PERIPH_BASEADDR+0X3800)
#define SPI3_BASEADDR						(APB1PERIPH_BASEADDR+0X3C00)

#define USART2_BASEADDR						(APB1PERIPH_BASEADDR+0X4400)
#define USART3_BASEADDR						(APB1PERIPH_BASEADDR+0X4800)
#define UART4_BASEADDR						(APB1PERIPH_BASEADDR+0X4C00)
#define UART5_BASEADDR						(APB1PERIPH_BASEADDR+0X5000)


/*
 * Base address of peripherals wich are hanging on APB2 bus
 */
#define EXTI_BASEADDR						(APB2PERIPH_BASEADDR+0X3C00)
#define SPI1_BASEADDR						(APB2PERIPH_BASEADDR+0X3000)
#define SYSCFG_BASEADDR						(APB2PERIPH_BASEADDR+0X3800)
#define USART1_BASEADDR						(APB2PERIPH_BASEADDR+0X1000)
#define USART6_BASEADDR						(APB2PERIPH_BASEADDR+0X1400)

/*
 * PERIPHERAL REGISTER DEFINITION STRUCTURES*************************************
 */

typedef struct
{
	__vo uint32_t MODER;		/*GPIO port mode register,						Address offset: 0x00*/
	__vo uint32_t OTYPER;		/*GPIO port output type register,				Address offset: 0x04*/
	__vo uint32_t OSPEEDR;		/*GPIO port output speed register,				Address offset: 0x08*/
	__vo uint32_t PUPDR;		/*GPIO port pull-up/pull-down register,			Address offset: 0x0C*/
	__vo uint32_t IDR;			/*GPIO port input data register,				Address offset: 0x10*/
	__vo uint32_t ODR;			/*GPIO port output data register				Address offset: 0x14*/
	__vo uint32_t BSRR;			/*GPIO port bit set/reset register				Address offset: 0x18*/
	__vo uint32_t LCKR;			/*GPIO port configuration lock register			Address offset: 0x1C*/
	__vo uint32_t AFR[2];		/*AFR[0]: GPIO alternate function low register, AFR[1]: GPIO alternate function high register			Address offset: 0x20-0X24*/
}GPIO_RegDef_t;

/*
 * peripheral register definition structure for RCC
 */

typedef struct
{
__vo uint32_t CR;						/*	Address offset: 0x00 */
__vo uint32_t PLLCFGR;					/*	Address offset: 0x04*/
__vo uint32_t CFGR;						/*	Address offset: 0x08*/
__vo uint32_t CIR;						/*	Address offset: 0x0C*/
__vo uint32_t AHB1RSTR;					/*	Address offset: 0x10*/
__vo uint32_t AHB2RSTR;					/*	Address offset: 0x14*/
__vo uint32_t AHB3RSTR;					/*	Address offset: 0x18*/
uint32_t	 RESERVED0;						/*	reserved, 0x1C*/
__vo uint32_t APB1RSTR;					/*	Address offset: 0x20*/
__vo uint32_t APB2RSTR;					/*	Address offset: 0x24*/
uint32_t 	RESERVED1[2];				/*	reserved, 0x28-0x2C*/
__vo uint32_t AHB1ENR;					/*	Address offset: 0x30*/
__vo uint32_t AHB2ENR;					/*	Address offset: 0x34*/
__vo uint32_t AHB3ENR;					/*	Address offset: 0x38*/
uint32_t 	RESERVED2;					/*	reserved 0x3C */
__vo uint32_t APB1ENR;					/*	Address offset: 0x40*/
__vo uint32_t APB2ENR;					/*	Address offset: 0x44*/
uint32_t 	RESERVED3[2];				/*	reserved, 0x48-0x4C */
__vo uint32_t AHB1LPENR;				/*	Address offset: 0x50*/
__vo uint32_t AHB2LPENR;				/*	Address offset: 0x54*/
__vo uint32_t AHB3LPENR;				/*	Address offset: 0x58*/

}RCC_RegDef_t;

/*
 * peripheral register definition structure for EXTI
 */


typedef struct
{
	__vo uint32_t IMR;				/*Interrupt mask register 				Address offset: 0x00*/
	__vo uint32_t EMR;				/*Event mask register					Address offset: 0x04*/
	__vo uint32_t RTSR;				/*Rising trigger selection register		Address offset: 0x08*/
	__vo uint32_t FTSR;				/*Falling trigger selection register	Address offset: 0x0C*/
	__vo uint32_t SWIER;			/*Software interrupt event register		Address offset: 0x10*/
	__vo uint32_t PR;				/*Pending register						Address offset: 0x14*/
}EXTI_RegDef_t;

/*
 *peripheral register definition structure  for SYSCFG
 */
typedef struct
{
	__vo uint32_t MEMRMP;					/*Address offset: 0x00*/
	__vo uint32_t PMC;						/*Address offset: 0x04*/
	__vo uint32_t EXTICR[4];				/*Address offset: 0x08-0x14*/ //16 pin divided 4
	uint32_t RESERVED1[2];					/*Address offset: 0x1C*/
	__vo uint32_t CMPCR;					/*Address offset: 0x20*/
	uint32_t RESERVED2[2];					/*Address offset: 0x28*/
	__vo uint32_t CFGR;						/*Address offset: 0x2C*/
}SYSCFG_RegDef_t;

/*
 *peripheral definitions (peripheral base address typecasted to xxx_RegDef_t)
 */

#define GPIOA								((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB								((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC								((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD								((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE								((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF								((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG								((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH								((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI								((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC									((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI 								((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG 								((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)


/*
 * Clock enable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()				(RCC->AHB1ENR |= (1<<0)) //GPIOA PERIPHERAL CLOCK ENABLE
#define GPIOB_PCLK_EN()				(RCC->AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN()				(RCC->AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN()				(RCC->AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN()				(RCC->AHB1ENR |= (1<<4))
#define GPIOF_PCLK_EN()				(RCC->AHB1ENR |= (1<<5))
#define GPIOG_PCLK_EN()				(RCC->AHB1ENR |= (1<<6))
#define GPIOH_PCLK_EN()				(RCC->AHB1ENR |= (1<<7))

/*
 * Clock enable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_EN()				(RCC->APB1ENR |= (1<<21))
#define I2C2_PCLK_EN()				(RCC->APB1ENR |= (1<<22))
#define I2C3_PCLK_EN()				(RCC->APB1ENR |= (1<<23))

/*
 * Clock enable Macros for SPIx peripherals
 */

#define SPI1_PCLK_EN()				(RCC->APB2ENR |= (1<<12))
#define SPI2_PCLK_EN()				(RCC->APB1ENR |= (1<<14))
#define SPI3_PCLK_EN()				(RCC->APB1ENR |= (1<<15))
#define SPI4_PCLK_EN()				(RCC->APB2ENR |= (1<<13))

/*
 * Clock enable Macros for USARTx peripherals
 */

#define USART1_PCLK_EN()				(RCC->APB2ENR |= (1<<4))
#define USART2_PCLK_EN()				(RCC->APB1ENR |= (1<<17))
#define USART3_PCLK_EN()				(RCC->APB1ENR |= (1<<18))
#define UART4_PCLK_EN()					(RCC->APB1ENR |= (1<<19))
#define UART5_PCLK_EN()					(RCC->APB1ENR |= (1<<20))
#define USART6_PCLK_EN()				(RCC->APB1ENR |= (1<<5))

/*
 * Clock enable Macros for SYSCFG peripherals
 */

#define SYSCFG_PCLK_EN()				(RCC->APB2ENR |= (1<<14))

/*
 * Clock disable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_DI()				(RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DI()				(RCC->AHB1ENR &= ~(1<<0))
#define GPIOC_PCLK_DI()				(RCC->AHB1ENR &= ~(1<<0))
#define GPIOD_PCLK_DI()				(RCC->AHB1ENR &= ~(1<<0))
#define GPIOE_PCLK_DI()				(RCC->AHB1ENR &= ~(1<<0))
#define GPIOF_PCLK_DI()				(RCC->AHB1ENR &= ~(1<<0))
#define GPIOG_PCLK_DI()				(RCC->AHB1ENR &= ~(1<<0))
#define GPIOH_PCLK_DI()				(RCC->AHB1ENR &= ~(1<<0))
/*
 * Clock disable Macros for I2Cx peripherals
 */

#define _PCLK_DI()				(RCC->APB2ENR |= (1<<4))
/*
 * Clock disable Macros for SPIx peripherals
 */

#define _PCLK_DI()				(RCC->APB2ENR |= (1<<4))

/*
 * Clock disable Macros for USARTx peripherals
 */

/*
 * Clock disable Macros for SYSCFG peripheral
 */

/*
 *macros to reset GPIOx peripherals
 */

#define GPIOA_REG_RESET()		do{ (RCC->AHB1RSTR |= (1<<0));	(RCC->AHB1RSTR &= ~(1<<0));} while(0) //do while condition zero loop: execute multiple C statement using single C macro
#define GPIOB_REG_RESET()		do{ (RCC->AHB1RSTR |= (1<<1));	(RCC->AHB1RSTR &= ~(1<<1));} while(0) //do while condition zero loop: execute multiple C statement using single C macro
#define GPIOC_REG_RESET()		do{ (RCC->AHB1RSTR |= (1<<2));	(RCC->AHB1RSTR &= ~(1<<2));} while(0) //do while condition zero loop: execute multiple C statement using single C macro
#define GPIOD_REG_RESET()		do{ (RCC->AHB1RSTR |= (1<<3));	(RCC->AHB1RSTR &= ~(1<<3));} while(0) //do while condition zero loop: execute multiple C statement using single C macro
#define GPIOE_REG_RESET()		do{ (RCC->AHB1RSTR |= (1<<4));	(RCC->AHB1RSTR &= ~(1<<4));} while(0) //do while condition zero loop: execute multiple C statement using single C macro
#define GPIOF_REG_RESET()		do{ (RCC->AHB1RSTR |= (1<<5));	(RCC->AHB1RSTR &= ~(1<<5));} while(0) //do while condition zero loop: execute multiple C statement using single C macro
#define GPIOG_REG_RESET()		do{ (RCC->AHB1RSTR |= (1<<6));	(RCC->AHB1RSTR &= ~(1<<6));} while(0) //do while condition zero loop: execute multiple C statement using single C macro
#define GPIOH_REG_RESET()		do{ (RCC->AHB1RSTR |= (1<<7));	(RCC->AHB1RSTR &= ~(1<<7));} while(0) //do while condition zero loop: execute multiple C statement using single C macro

#define GPIO_BASEADDR_TO_CODE(x)	(	(x==GPIOA) ? 0 :\
										(x==GPIOB) ? 1 :\
										(x==GPIOC) ? 2 :\
										(x==GPIOD) ? 3 :\
										(x==GPIOE) ? 4 :\
										(x==GPIOF) ? 5 :\
										(x==GPIOG) ? 6 :\
										(x==GPIOH) ? 7 :0	)

/*
 *IRQ(Interrupt Request) Numbers of STM32F407x MCU
 *
 */
#define IRQ_NO_EXTI0				6 				//pin 0
#define IRQ_NO_EXTI1				7				//pin 1
#define IRQ_NO_EXTI2				8				//pin 2
#define IRQ_NO_EXTI3				9				//pin 3
#define IRQ_NO_EXTI4				10				//pin 4
#define IRQ_NO_EXTI9_5				23				//pin 5,6,7,8,9
#define IRQ_NO_EXTI15_10			40				//pin 10,11,12,13,14,15

/*
 * macros for all the possible priority levels
 */
#define NVIC_IRQ_PRI0				0
#define NVIC_IRQ_PRI15				15

//some generic Macros

#define ENABLE 				1
#define DISABLE 			0
#define SET 				ENABLE
#define RESET 				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET


#include "stm32f407xx_ gpio_driver.h"


#endif /* INC_STM32F407XX_H_ */
