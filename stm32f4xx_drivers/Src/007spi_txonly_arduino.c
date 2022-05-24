/*
 * 007spi_txonly_arduino.c
 *
 *  Created on: May 7, 2022
 *      Author: Bach
 */


#include<string.h>
#include "stm32f411xx.h"

void SPI2_GPIOInits(void){
	GPIO_Handle_t SPIPins;
	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	// MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	// MISO
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	//GPIO_Init(&SPIPins);

	// NSS
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	//GPIO_Init(&SPIPins);

}

void SPI2_Inits(){
	SPI_Handle_t SPI2handle;
	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8; // 8Mhz
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_DI;

	SPI_Init(&SPI2handle);
}

void GPIO_ButtonInit(void){
	GPIO_Handle_t GpioBtn;

	GpioBtn.pGPIOx = GPIOA;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode   = GPIO_MODE_INPUT;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GpioBtn);
}

void delay(void){
	for (uint32_t i =0 ; i < 500000/2; i++);
}

int main (void){
	char user_data[] = "Hello world";
	//this function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	SPI2_Inits();


	/* making SSOE 1 does NSS output enable.
	 * The NSS pin is automatically managed by the hardware
	 * i.e when SPE=1, NSS will be pulled to low
	 * and NSS pin will be high when SPE = 0
	 */

	SPI_SSOEConfig(SPI2,ENABLE);
	while(1){

				//wait till button is pressed
				while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0) );

				//to avoid button de-bouncing related issues 200ms of delay
				delay();
			// enable the SPI2 peripheral

			SPI_PeripheralControl(SPI2,ENABLE);
			// first send length information
			uint8_t datalen = strlen(user_data);
			SPI_SendData(SPI2,&datalen,1);
			// to send data
			SPI_SendData(SPI2,(uint8_t*)user_data,strlen(user_data));


			while( SPI_Get_Flag_Status(SPI2,SPI_BUSY_FLAG) );


			SPI_PeripheralControl(SPI2,DISABLE);

	}



	return 0;
}
