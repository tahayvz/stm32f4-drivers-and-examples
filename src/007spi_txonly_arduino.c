/*
 * 007spi_txonly_arduino.c
 *
 *  Created on: 21 May 2019
 *      Author: tahay
 */


#include "stm32f407xx.h"
#include <string.h>

void delay()
{
	for(uint32_t i=0; i<500000/2; i++);
}

void SPI2_GPIOInits(void){
	GPIO__Handle_t SPIPins;

	SPIPins.pGPIOx=GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode= 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP; //open drain not required for SPI, required for I2C
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PIN_PU;//we have slave so used pull up because it may receive some junk data
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	//MISO
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_14;
	//GPIO_Init(&SPIPins);

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);

}

SPI2_Inits(){

	SPI_Handle_t SPI2handle;
	SPI2handle.pSPIx=SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig=SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode=SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed= SPI_SCLK_SPEED_DIV8; //generates SCLK of 2MHz
	SPI2handle.SPIConfig.SPI_DFF=SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL=SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA=SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM=SPI_SSM_DI; //hardware slave management enabled for NSS pin
}
void GPIO_ButtonInit()
{
	GPIO__Handle_t GPIOBtn;
	GPIOBtn.pGPIOx=GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD; //there is already pull down resistor in button pin

	GPIO_Init(&GPIOBtn);
}

int main(void){

	char user_data[]= "An arduino uno board is best suited for beginners. Arduino mega board is for enthusiastic who require a lot of I/O pins";

	GPIO_ButtonInit();

	//this function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	//this function is used to initialize the SPI2 peripheral parameters
	SPI2_Inits();

	/*
	 * NSS output will be enabled when SSOE=1
	 * NSS pin is automatically managed by the hardware
	 * i.e. SPE=1, NSS will be pulled to low and NSS pin will be high when SPE=0
	 * SSM=0 SPE=1 NSS output will be 0
	 * SSM=0 SPE=0 NSS output will be 1
	 * */

	SPI_SSOEConfig(SPI2,ENABLE);

	while(1)
	{
		//wait till button is pressed
		while(!GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0));//when pressed btn function return 1

		delay(200);

		//enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		//first send length information
		uint8_t dataLen= strlen(user_data);
		SPI_SendData(SPI2,&dataLen,1);

		//to send data
		SPI_SendData(SPI2,(uint8_t*)user_data,strlen(user_data));//arduino sketch expects 1 byte of length followed by data

		//lets confirm SPI is not busy
		while(SPI_GetFlagStatus(SPI2,SPI_SR_BSY));

		//disable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, DISABLE);
	}

	return 0;

}
