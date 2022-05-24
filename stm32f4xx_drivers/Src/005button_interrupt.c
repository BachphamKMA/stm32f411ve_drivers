/*
 * 002led_button.c
 *
 *  Created on: Apr 30, 2022
 *      Author: Bach
 */

#include <string.h>
#include "stm32f411xx.h"
#include "stm32f411xx_gpio_driver.h"


void delay(void){
	// cai nay` se tao delay 200ms khi system clock la 16MHz
	for (uint32_t i =0 ; i < 500000/2; i++);


}
int main (void){
	GPIO_Handle_t GpioLed, GpioBtn;
	memset(&GpioLed,0,sizeof(GpioLed)); // cho gia tri ve 0 cua struct
	memset(&GpioBtn,0,sizeof(GpioBtn));

// LED
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode   = GPIO_MODE_OUTPUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&GpioLed);
// nut an
	GpioBtn.pGPIOx = GPIOD;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode   = GPIO_MODE_IT_FT;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;


	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&GpioBtn);

	// IRQ configurations
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, 15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);

while(1);
}

void EXTI9_5_IRQHandler(void){
	delay();// wait till button de-boucing gets over
	GPIO_IRQHandling(GPIO_PIN_NO_5); // clear the peding event from  exti line
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12); // toggle LED
}


