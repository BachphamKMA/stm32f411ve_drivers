/*
 * stm32f411xx_gpio_driver.c
 *
 *  Created on: Apr 26, 2022
 *      Author: Bach
 */


#include "stm32f411xx_gpio_driver.h"

/*
 *     APIs hỗ trợ bởi driver
 *
 */


/*
 *  Cài đặt Clock cho ngoại vi
 */

/*********************************************************
 *  @fn				- 	GPIO_Init
 *
 * 	@brief			-	Hàm này sẽ bật và tắt clock ngoại vi cho port GPIO đã cho
 *
 * 	@param[in]		- 	địa chỉ gốc của thiết bị ngoại vi
 * 	@param[in]		-	Bật hoặc tắt macros
 * 	@param[in]		-
 *
 * @return			-	none
 *
 * @Note			-	none
 *
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}else if (pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}else if (pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		}else if (pGPIOx == GPIOD){
			GPIOD_PCLK_EN();
		}else if (pGPIOx == GPIOE){
			GPIOE_PCLK_EN();
		}else if (pGPIOx == GPIOH){
			GPIOH_PCLK_EN();
		}
	}else{
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_DI();
		}else if (pGPIOx == GPIOB){
			GPIOB_PCLK_DI();
		}else if (pGPIOx == GPIOC){
			GPIOC_PCLK_DI();
		}else if (pGPIOx == GPIOD){
			GPIOD_PCLK_DI();
		}else if (pGPIOx == GPIOE){
			GPIOE_PCLK_DI();
		}else if (pGPIOx == GPIOH){
			GPIOH_PCLK_DI();
		}
	}
}



void GPIO_Init(GPIO_Handle_t *pGPIOHandle){
	uint32_t temp = 0;

	// bat gpio clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	// 1. Cài đặt chế độ cho gpio pin
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){

		// dưới chế độ Analog thì không phải là chế độ ngắt

		temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
		pGPIOHandle->pGPIOx->MODER &= ~( 0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing đảo "~" của 0x3(11) là (00) để reset trạng thái
		pGPIOHandle->pGPIOx->MODER |= temp; // setting
		temp = 0;
	}else{

		// chế độ ngắt
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT){
			//1.Configure the FTSR
			EXTI->FTSR  |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// CLEAR the corresponding RTSR bit
			EXTI->RTSR  &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){
			//1.Configure the RTSR
			EXTI->RTSR  |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// CLEAR the corresponding RTSR bit
			EXTI->FTSR  &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT){
			//1.Configure the RTSR and FTSR
			EXTI->RTSR  |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// CLEAR the corresponding RTSR bit
			EXTI->FTSR  |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//2. configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);
		//3. configure the exti interrupt delivery using IMR
		EXTI->IMR |=  1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	}

		temp = 0;
	// 2. Cài đặt tốc độ
		temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
		pGPIOHandle->pGPIOx->OSPEEDR &= ~( 0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->OSPEEDR |= temp;
		temp = 0;
	// 3. Cài đặt kéo lên hay kéo xuống

		temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
		pGPIOHandle->pGPIOx->PUPDR &= ~( 0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->PUPDR |= temp;
		temp = 0;
	// 4. Cài đặt đầu ra
		temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
		pGPIOHandle->pGPIOx->OTYPER &= ~( 0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->OTYPER |= temp;

		temp = 0;
	// 5. Cài đặt các chức năng khác
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){
			// cài đặt thanh ghi chức năng khác
			uint8_t temp1, temp2;
			temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
			temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
			pGPIOHandle->pGPIOx->AFR[temp1] &= ~( 0xF << (4 * temp2) ); // clearing
			pGPIOHandle->pGPIOx->AFR[temp1] |= ( pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2) );
		}

}
void GPIO_DeInint(GPIO_RegDef_t *pGPIOx){

		if(pGPIOx == GPIOA){
			GPIOA_REG_RESET();
		}else if (pGPIOx == GPIOB){
			GPIOB_REG_RESET();
		}else if (pGPIOx == GPIOC){
			GPIOC_REG_RESET();
		}else if (pGPIOx == GPIOD){
			GPIOD_REG_RESET();
		}else if (pGPIOx == GPIOE){
			GPIOE_REG_RESET();
		}else if (pGPIOx == GPIOH){
			GPIOH_REG_RESET();
		}

}

/*
 * Đọc và viết data
 */

/*********************************************************
 *  @fn				-
 *
 * 	@brief			-
 *
 * 	@param[in]		-
 * 	@param[in]		-
 * 	@param[in]		-
 *
 * @return			-		0 or 1
 *
 * @Note			-
 *
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	uint8_t value;
	value = (uint8_t) (( pGPIOx->IDR >> PinNumber) & 0x00000001 ); // dich phải để lấy bit ở LSB
	return value;
}
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){
	uint16_t value;

	value = (uint16_t)pGPIOx->IDR;

	return value;


}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET){
		// write 1 to the output data register at the bit field corresponding pin number
		pGPIOx->ODR |= ( 1 << PinNumber);
	}else{
		// write 0
		pGPIOx->ODR &= ~( 1 << PinNumber);
	}
}
/*********************************************************
 *  @fn				-		WriteToOutputPort
 *
 * 	@brief			-
 *
 * 	@param[in]		-
 * 	@param[in]		-
 * 	@param[in]		-
 *
 * @return			-
 *
 * @Note			-
 *
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value){
	pGPIOx->ODR	= Value;

}


void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	pGPIOx->ODR	^= ( 1 << PinNumber);

}

/*
 * Cài đặt và điều khiển IRQ
 */

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

	if(EnorDi == ENABLE){
		if(IRQNumber <= 31){
			// program ISER0 register
			*NVIC_ISER0	|= ( 1 << IRQNumber);
		}else if(IRQNumber > 31 && IRQNumber < 64){
			// program ISER1 register // 32 to 63
			*NVIC_ISER1	|= ( 1 << (IRQNumber % 32) );
		}else if(IRQNumber >= 64 && IRQNumber < 96){
			// program ISER2 register // 64 to 95
			*NVIC_ISER2	|= ( 1 << (IRQNumber % 64) );
		}
	}else{
		if(IRQNumber <= 31)
		{
			*NVIC_ICER0	|= ( 1 << IRQNumber);
		}else if(IRQNumber > 31 && IRQNumber < 64){
			*NVIC_ICER1	|= ( 1 << (IRQNumber % 32) );
		}else if(IRQNumber >= 6 && IRQNumber < 96){
			*NVIC_ICER2	|= ( 1 << (IRQNumber % 64) );
		}

	}
}
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx ) |= ( IRQPriority << shift_amount);


}
void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clear the exti pr register corresponding to the pin number
	if(EXTI->PR & ( 1 << PinNumber )){
		// clear
		EXTI->PR |= (1 << PinNumber);
	}
}



