/*
 * 006spi_tx_testing.c
 *
 *  Created on: 20 May 2019
 *      Author: tahay
 */
#include "stm32f407xx.h"
#include <string.h>

/*
 *
PB15-->SPI2_MOSI
PB14-->SPI2_MISO
PB13-->SPI2_SCLK
PB12-->SPI2_NSS
AF(ALTERNATE FUNCTIONALITY MODE): 5
*/

void SPI2_GPIOInits(void){
	GPIO__Handle_t SPIPins;

	SPIPins.pGPIOx=GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode= 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP; //open drain not required for SPI, required for I2C
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
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
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_12;
	//GPIO_Init(&SPIPins);
	//disabled MISONSS pins because there is no slave
}

SPI2_Inits(){

	SPI_Handle_t SPI2handle;
	SPI2handle.pSPIx=SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig=SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode=SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed= SPI_SCLK_SPEED_DIV2;
	SPI2handle.SPIConfig.SPI_DFF=SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL=SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA=SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM=SPI_SSM_EN; //no use slave so software slave management enabled for NSS pin
}
int main(void){

	char user_data[]= "Hello World";
	//this function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();
	//this function is used to initialize the SPI2 peripheral parameters
	SPI2_Inits();
	//This makes NSS signal internally high and avoids MODF error. enable peripheral for solution of fault error. SSI=1
	SPI_SSIConfig(SPI2,ENABLE);

	//enable the SPI2 peripheral
	SPI_PeripheralControl(SPI2, ENABLE);

	//to send data
	SPI_SendData(SPI2,(uint8_t*)user_data,strlen(user_data));

	//lets confirm SPI is not busy
	while(SPI_GetFlagStatus(SPI2,SPI_SR_BSY));

	//disable the SPI2 peripheral. When SPIis disabled SCLK and DATA will HIGH. HIGH is idle time
	SPI_PeripheralControl(SPI2, DISABLE);

	while(1);

	return 0;
}
