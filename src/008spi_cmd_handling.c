/*
 * 008spi_cmd_handling.c
 *
 *  Created on: 21 May 2019
 *      Author: tahay
 */

#include "stm32f407xx.h"
#include <string.h>

//command codes
#define COMMAND_LED_CTRL		0x50
#define COMMAND_SENSOR_READ		0x51
#define COMMAND_LED_READ		0x52
#define COMMAND_PRINT			0x53
#define COMMAND_ID_READ			0x54

#define LED_ON			1
#define LED_OFF			0

//arduino analog pins
#define ANALOG_PIN0		0
#define ANALOG_PIN1		1
#define ANALOG_PIN2		2
#define ANALOG_PIN3		3
#define ANALOG_PIN4		4

//arduino led
#define LED_PIN			9

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
	SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);

}

void SPI2_Inits(){

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
	GPIO__Handle_t GpioLed,GPIOBtn;
	GPIOBtn.pGPIOx=GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD; //there is already pull down resistor in button pin

	GPIO_Init(&GPIOBtn);

	//this is led gpio configuration
	GpioLed.pGPIOx=GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_OD;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD,ENABLE);
	GPIO_Init(&GpioLed);
}

int SPI_verifyResponse(int ackbyte)
{
	if(ackbyte==0xF5)
	{
		//ack
		return 1;
	}
	return 0; //nack
}

int main(void){

	uint8_t dummy_write = 0xff;
	uint8_t dummy_read;

	GPIO_ButtonInit();

	//this function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	//this function is used to initialize the SPI2 peripheral parameters
	SPI2_Inits();

	/*
	 * making SSOE 1 does NSS output enable
	 * NSS pin is automatically managed by the hrdware
	 * i.e. when SPE=1, NSS will be pulled to low and NSS pin will be high when SPE=0*/
	SPI_SSOEConfig(SPI2,ENABLE);

	while(1)
	{
		//wait till button is pressed
		while(!GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0));//when pressed btn function return 1

		delay();

		//enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		//1. CMD_LED_CTRL <pin no 1> <value 1>
		uint8_t commandcode=COMMAND_LED_CTRL;
		uint8_t ackbyte;
		uint8_t args[2];

		//send command
		SPI_SendData(SPI2,&commandcode,1);//SPI communication when master or slave sends 1 byte, it also receive 1 byte. This transmission od 1 byte resulted 1 garbage byte collection in Rx buffer of master and RXNE flag is set. So dummy read and clear the flag
		/*when slave receive it will check supported or not. If support send ACK. SPI not initiate the data transfer
		 * so you have to send some dummy byte to SPI and have to get that response back. Now slave ready for ACK or NACK
		 * byte in its shift register. If you want move data out of the slave shift register you have to send dummy byte*/

		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2,&dummy_read,1);


		//send dummy bits(1 byte) to fetch the response(acknowledgement) from the slave
		SPI_SendData(SPI2,&dummy_write,1);

		//read the ack byte received
		SPI_ReceiveData(SPI2,&ackbyte,1);

		if(SPI_verifyResponse(ackbyte)) //check bits correct or not
		{
			args[0]=LED_PIN;
			args[1]=LED_ON;

			//send arguments
			SPI_SendData(SPI2,args,2);
		}
		//end of COMMAND_LED_CTRL

		//2.CMD_SENSOR_READ <analog pin number 1>
		//wait till button is pressed
		while(!GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0));//when pressed btn function return 1

		//to avoid button de-bouncing released issues 200ms of delay
		delay();

		commandcode=COMMAND_SENSOR_READ;

		//send command
		SPI_SendData(SPI2,&commandcode,1);

		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2,&dummy_read,1);

		//send dummy bits(1 byte) to fetch the response from the slave
		SPI_SendData(SPI2,&dummy_write,1);

		//read the ack byte received
		SPI_ReceiveData(SPI2,&ackbyte,1);

		if(SPI_verifyResponse(ackbyte))//check bits correct or not
		{
			args[0]=ANALOG_PIN0;

			//send arguments
			SPI_SendData(SPI2,args,1); //sending 1 byte. //send always result in reception.that's why we are again reading data register to clear of RXE


		}

		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2,&dummy_read,1);

		//insert some delay so that slave can ready with the data. Slave takes some time to read analog value
		delay();

		//send dummy bits(1 byte) to fetch the response from the slave
		SPI_SendData(SPI2,&dummy_write,1);

		uint8_t analog_read;
		SPI_ReceiveData(SPI2,&analog_read,1);


		//lets confirm SPI is not busy
		while(SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG));

		//disable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, DISABLE);

	}

	return 0;

}
