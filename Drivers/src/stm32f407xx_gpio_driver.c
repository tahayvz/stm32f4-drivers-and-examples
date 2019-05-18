/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: 18 May 2019
 *      Author: tahay
 */

#include "stm32f407xx_ gpio_driver.h"

/*
 *Peripheral clock setup. GPIO enable disable
 */

/***********************************************************************
 *	@fn					-GPIO_PeriClockControl
 *
 *	@brief				-this function enables or disables peripheral clock for the given GPIO port
 *
 *	@param[in]			-base address of the GPIO peripheral
 *	@param[in]			-ENABLE or DISABLE macros
 *	@param[in]			-
 *
 *	@return				-none
 *
 *	@note				-none
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi){

	if(EnorDi ==ENABLE)
	{
		if(pGPIOx==GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if(pGPIOx==GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if(pGPIOx==GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if(pGPIOx==GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if(pGPIOx==GPIOE)
		{
			GPIOE_PCLK_EN();
		}else if(pGPIOx==GPIOF)
		{
			GPIOF_PCLK_EN();
		}else if(pGPIOx==GPIOG)
		{
			GPIOG_PCLK_EN();
		}else if(pGPIOx==GPIOH)
		{
			GPIOH_PCLK_EN();
		}
	}
	else{
		if(pGPIOx==GPIOA)
		{
		GPIOA_PCLK_DI();
		}else if(pGPIOx==GPIOB)
		{
		GPIOB_PCLK_DI();
		}else if(pGPIOx==GPIOC)
		{
		GPIOC_PCLK_DI();
		}else if(pGPIOx==GPIOD)
		{
		GPIOD_PCLK_DI();
		}else if(pGPIOx==GPIOE)
		{
		GPIOE_PCLK_DI();
		}else if(pGPIOx==GPIOF)
		{
		GPIOF_PCLK_DI();
		}else if(pGPIOx==GPIOG)
		{
		GPIOG_PCLK_DI();
		}else if(pGPIOx==GPIOH)
		{
		GPIOH_PCLK_DI();
		}
		}
}

/*
 *
 */
/***********************************************************************
 *	@fn					-GPIO_Init
 *
 *	@brief				-
 *
 *	@param[in]			-
 *	@param[in]			-
 *	@param[in]			-
 *
 *	@return				-none
 *
 *	@note				-none
 */
void GPIO_Init(GPIO__Handle_t *pGPIOHandle){

	uint32_t temp=0; 	//temp register

	//1. configure the mode of gpio pin

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode<=GPIO_MODE_ANALOG)
	{
		//the non interrup mode
		temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); 	//each pin take 2 bit
		pGPIOHandle->pGPIOx->MODER &= ~(0x3<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing bit non 11 will be 00
		pGPIOHandle->pGPIOx->MODER |=temp; //setting
	}else
	{
		//interrupt mode
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_IT_FT)
		{
			//configure the FTSR of EXTI
			EXTI->FTSR |= (1 <<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clear the corresponding RTSR bit
			EXTI->RTSR &= ~(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_IT_RT)
		{
			//configure the RTSR
			EXTI->RTSR |= (1 <<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clear the corresponding FTSR bit
			EXTI->FTSR &= ~(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_IT_RFT)
		{
			//configure both FTSR and RTSR
			EXTI->RTSR |= (1 <<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//2. configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber /4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber %4;
		uint8_t portcode= GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1]= portcode << (temp *4);

		//3. enable EXTI into delivery using IMR
		EXTI->IMR |= 1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	}
	temp=0;

	//2. configure the speed
	temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing bit non 11 will be 00
	pGPIOHandle->pGPIOx->OSPEEDR|=temp;
	temp=0;

	//3. configure the pull up pull down settings
	temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing bit non 11 will be 00
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp=0;

	//4. configure the output type
	temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing bit non 1 will be 0
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp=0;

	//5. configure the alt functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){
		//configure the alternate function registers
		uint32_t temp1, temp2;
		temp1=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/8;
		temp2=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF <<(4*temp2)); //4 bit each pin
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode <<(4*temp2));
	}
}

/***********************************************************************
 *	@fn					-GPIO_DeInit
 *
 *	@brief				-
 *
 *	@param[in]			-
 *	@param[in]			-
 *	@param[in]			-
 *
 *	@return				-none
 *
 *	@note				-none
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){

			if(pGPIOx==GPIOA)
			{
				GPIOA_REG_RESET();
			}else if(pGPIOx==GPIOB)
			{
				GPIOA_REG_RESET();
			}else if(pGPIOx==GPIOC)
			{
				GPIOA_REG_RESET();
			}else if(pGPIOx==GPIOD)
			{
				GPIOA_REG_RESET();
			}else if(pGPIOx==GPIOE)
			{
				GPIOA_REG_RESET();
			}else if(pGPIOx==GPIOF)
			{
				GPIOA_REG_RESET();
			}else if(pGPIOx==GPIOG)
			{
				GPIOA_REG_RESET();
			}else if(pGPIOx==GPIOH)
			{
				GPIOA_REG_RESET();
			}else if(pGPIOx==GPIOI)
			{
				GPIOA_REG_RESET();
			}

}

/***********************************************************************
 *	@fn					-GPIO_ReadFromInputPin
 *
 *	@brief				-
 *
 *	@param[in]			-
 *	@param[in]			-
 *	@param[in]			-
 *
 *	@return				-0 or 1
 *
 *	@note				-none
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

	uint8_t value;
	value=(uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001); //shift IDR0 posisition for read
	return value; //return 0 or 1
}

/***********************************************************************
 *	@fn					-GPIO_ReadFromInputPort
 *
 *	@brief				-
 *
 *	@param[in]			-
 *	@param[in]			-
 *	@param[in]			-
 *
 *	@return				-
 *
 *	@note				-none
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){
	uint16_t value;
	value=(uint16_t)pGPIOx->IDR;
	return value; //return entire port IDR

}

/***********************************************************************
 *	@fn					-GPIO_WritetoOutputPin
 *
 *	@brief				-
 *
 *	@param[in]			-
 *	@param[in]			-
 *	@param[in]			-
 *
 *	@return				-
 *
 *	@note				-write value to PinNumber
 */

void GPIO_WritetoOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value){

	if(Value == GPIO_PIN_SET)
	{
		//write 1 to the output data register at the bit field corresponding o the PinNumber
		pGPIOx->ODR |=(1<<PinNumber);
	}else{
		//write 0
		pGPIOx->ODR &= ~(1<<PinNumber); //clearing bit position corresponding to the pinnumber in ODR
	}
}

/***********************************************************************
 *	@fn					-GPIO_WritetoOutputPort
 *
 *	@brief				-
 *
 *	@param[in]			-
 *	@param[in]			-
 *	@param[in]			-
 *
 *	@return				-
 *
 *	@note				-copy Value to pGPIOx->ODR
 */

void GPIO_WritetoOutputPort(GPIO_RegDef_t *pGPIOx, uint8_t Value){

	pGPIOx->ODR=Value;
}
/***********************************************************************
 *	@fn					-GPIO_ToggleOutputPin
 *
 *	@brief				-
 *
 *	@param[in]			-
 *	@param[in]			-
 *	@param[in]			-
 *
 *	@return				-
 *
 *	@note				-toggle required bit field
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

	pGPIOx->ODR ^= (1<<PinNumber); //bitwise XOR
}

/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQConfig(uint8_t IRQPriority, uint8_t EnorDi){

}
void GPIO_IRQHandling(uint8_t PinNumber){

}



