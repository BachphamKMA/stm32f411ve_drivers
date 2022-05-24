/*
 * stm32f411xx_rcc_driver.h
 *
 *  Created on: May 24, 2022
 *      Author: Bach
 */

#ifndef INC_STM32F411XX_RCC_DRIVER_H_
#define INC_STM32F411XX_RCC_DRIVER_H_

#include "stm32f411xx.h"

// Trả về giá trị của clock ở APB1
uint32_t RCC_GetPCLK1Value(void);

// Trả về giá trị của clock ở APB2
uint32_t RCC_GetPCLK2Value(void);


uint32_t  RCC_GetPLLOutputClock(void);

#endif /* INC_STM32F411XX_RCC_DRIVER_H_ */
