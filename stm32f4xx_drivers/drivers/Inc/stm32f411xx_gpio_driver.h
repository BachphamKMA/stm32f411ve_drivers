
/*
 * stm32f411xx_gpio_driver.h
 *
 *  Created on: Apr 23, 2022
 *      Author: Bach
 */

#ifndef INC_STM32F411XX_GPIO_DRIVER_H_
#define INC_STM32F411XX_GPIO_DRIVER_H_


#include "stm32f411xx.h"

typedef struct
{
	uint8_t GPIO_PinNumber;		/*!< possibles values from @GPIO_PIN_NUMBERS > */
	uint8_t GPIO_PinMode;      /*!< possibles values from @GPIO_PIN_MODES > */
	uint8_t GPIO_PinSpeed;		/*!< possibles values from @GPIO_PIN_SPEED > */
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;
/*
 * This is a Handle structure for a GPIO pin
 */
typedef struct
{

	GPIO_RegDef_t *pGPIOx;				// bien nay la bien con tro
	GPIO_PinConfig_t GPIO_PinConfig;

}GPIO_Handle_t;

/*
 *	@GPIO_PIN_NUMBERS
 *	GPIO pin numbers
 */

#define GPIO_PIN_NO_0	0
#define GPIO_PIN_NO_1	1
#define GPIO_PIN_NO_2	2
#define GPIO_PIN_NO_3	3
#define GPIO_PIN_NO_4	4
#define GPIO_PIN_NO_5	5
#define GPIO_PIN_NO_6	6
#define GPIO_PIN_NO_7	7
#define GPIO_PIN_NO_8	8
#define GPIO_PIN_NO_9	9
#define GPIO_PIN_NO_10	10
#define GPIO_PIN_NO_11	11
#define GPIO_PIN_NO_12	12
#define GPIO_PIN_NO_13	13
#define GPIO_PIN_NO_14	14
#define GPIO_PIN_NO_15	15

/*
 * @GPIO_PIN_MODES
 * Các chế độ của GPIO
 */

#define GPIO_MODE_INPUT  0
#define GPIO_MODE_OUTPUT 1
#define GPIO_MODE_ALTFN  2
#define GPIO_MODE_ANALOG 3
#define GPIO_MODE_IT_FT  4 // interupt khi o falling edge
#define GPIO_MODE_IT_RT	 5 // interup khi o rising edge
#define GPIO_MODE_IT_RFT 6 // ______________ rising falling edge

/*
 * Các chế độ output của pin GPIO
 */
#define GPIO_OP_TYPE_PP 0
#define GPIO_OP_TYPE_OD	1

/*
 * @GPIO_PIN_SPEED
 * Các tốc độ của pin GPIO
 */
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM 	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH 	3


/*
 * Macros cho cài đặt kéo lên và kéo xuống của GPIO
 */
#define GPIO_NO_PUPD 0
#define GPIO_PIN_PU	1
#define GPIO_PIN_PD 2



/*
 *     APIs hỗ trợ bởi driver
 *
 */

/*
 *  Cài đặt Clock cho ngoại vi
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);
/*
 * Init và Deinit
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInint(GPIO_RegDef_t *pGPIOx);

/*
 * đọc và viết data
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx); // port la 16 pin co the dung 16
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * Cài đặt và điều khiển IRQ
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32F411XX_GPIO_DRIVER_H_ */
