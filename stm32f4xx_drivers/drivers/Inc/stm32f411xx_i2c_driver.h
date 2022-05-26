/*
 * stm32f411xx_i2c_driver.h
 *
 *  Created on: May 14, 2022
 *      Author: Bach
 */

#ifndef INC_STM32F411XX_I2C_DRIVER_H_
#define INC_STM32F411XX_I2C_DRIVER_H_

#include "stm32f411xx.h"

/*
 * Cài đặt struct cho ngoại vi của I2C
 */

typedef struct{
	uint32_t I2C_SCLSpeed;
	uint32_t I2C_DeviceAddress;
	uint32_t I2C_ACKControl;
	uint32_t I2C_FMDutyCycle;
}I2C_Config_t;

/*
 * Khởi tạo xử lý cho struct của ngoại vi I2C
 */

typedef struct{
	I2C_RegDef_t 	*pI2Cx;
	I2C_Config_t	I2C_Config;
	uint8_t 		*pTxBuffer;	/* To store the app. Tx buffer address */
	uint8_t			*pRxBuffer; /* To store the app. Rx buffer address */
	uint32_t		TxLen;		/* To store Tx len */
	uint32_t		RxLen;		/* To store Rx len */
	uint8_t			TxRxState;	/* To store communication state */
	uint8_t			DevAddr;	/* To store slave/device address */
	uint32_t		RxSize;		/* To store Rx size */
	uint8_t			Sr;			/* To store repeated start value */
}I2C_Handle_t;

/*
 * @I2C_SCLSpeed
 */

#define I2C_SCL_SPEED_SM 100000
#define I2C_SCL_SPEED_FM4K 400000
#define I2C_SCL_SPEED_FM2K 200000


 /*
  * @I2C_AckControl
  */

#define I2C_ACK_ENABLE		1
#define I2C_ACK_DISABLE		0

/*
 * @I2C_FMDutyCycle
 */

#define I2C_FM_DUTY_2		0
#define I2C_FM_DUTY__16_9	1

/*
 * I2C related status flags definitions
 */

#define I2C_TXE_FLAG 		(1 << I2C_SR1_TxE)
#define I2C_RXNE_FLAG 		(1 << I2C_SR1_RxNE)
#define I2C_SB_FLAG			(1 << I2C_SR1_SB)
#define I2C_ADDR_FLAG		(1 << I2C_SR1_ADDR)
#define I2C_BTF_FLAG		(1 << I2C_SR1_BTF)
#define I2C_ADD10_FLAG		(1 << I2C_SR1_ADD10)
#define I2C_STOPF_FLAG		(1 << I2C_SR1_STOPF)
#define I2C_BERR_FLAG		(1 << I2C_SR1_BERR)
#define I2C_ARLO_FLAG		(1 << I2C_SR1_ARLO)
#define I2C_AF_FLAG			(1 << I2C_SR1_AF)
#define I2C_OVR_FLAG		(1 << I2C_SR1_OVR)
#define I2C_TIMEOUT_FLAG	(1 << I2C_SR1_TIMEOUT)

#define I2C_DISABLE_SR	RESET
#define I2C_ENABLE_SR		SET

/*
 * I2C applicatoin events macros
 */

#define I2C_EV_TX_CMPLT		0
#define I2C_EV_RX_CMPLT		1
#define I2C_EV_STOP		2
#define I2C_ERROR_BERR  	3
#define I2C_ERROR_ARLO  	4
#define I2C_ERROR_AF    	5
#define I2C_ERROR_OVR   	6
#define I2C_ERROR_TIMEOUT 	7
#define I2C_EV_DATA_REQ		8
#define I2C_EV_DATA_RCV		9


/*
 * I2C application states
 */

#define I2C_READY				0
#define I2C_BUSY_IN_RX				1
#define I2C_BUSY_IN_TX				2



void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);


/*
 * Init và Deinit
 */
void I2C_Init(I2C_Handle_t*pI2CHandle);
void I2C_DeInint(I2C_RegDef_t *pI2Cx);

/*
 * Data send and receive
 */


void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr);

uint8_t  I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len,uint8_t SlaveAddr,uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr);

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);

void I2C_SlaveSendData(I2C_RegDef_t *pI2C,uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C);
/*
 * Cài đặt và điều khiển IRQ
 */

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);

/*
 * Các API điều khiển ngoại vi khác
 */

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);
uint8_t I2C_Get_Flag_Status(I2C_RegDef_t *pI2Cx, uint32_t FlagName);
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
/*
 * Application callback
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv);

#endif /* INC_STM32F411XX_I2C_DRIVER_H_ */
