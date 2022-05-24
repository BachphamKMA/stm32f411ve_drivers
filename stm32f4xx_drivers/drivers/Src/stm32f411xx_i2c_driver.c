/*
 * stm32f411xx_i2c_driver.c
 *
 *  Created on: May 14, 2022
 *      Author: Bach
 */


#include "stm32f411xx_i2C_driver.h"



static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle);


static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= ( 1 << I2C_CR1_START);
}

static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr){
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1); // clear bit 0 ( cần clear bit 0 để gửi yêu cầu viết ) đây chính là bit Read or Write
	pI2Cx->DR = SlaveAddr;
}



static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle){
	// Check for device mode ( master / salve)
	uint32_t dummy_read;
	if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL)){
		// device mode in master mode
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX){

			if(pI2CHandle->RxSize == 1){
				// first disable the ack
				I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

				// clear the ADDR flag, read SR1, read SR2)
				dummy_read = pI2CHandle->pI2Cx->SR1;
				dummy_read = pI2CHandle->pI2Cx->SR2;
				(void)dummy_read;


			}
		}
	}else{
		//device is in slave mode
	}
}



 void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx){
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr){
	SlaveAddr = SlaveAddr << 1 ;
	SlaveAddr |= 1;
	pI2Cx->DR = SlaveAddr;
}


void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){
	if(EnorDi == I2C_ACK_ENABLE){
		pI2Cx->CR1 |= ( 1 << I2C_CR1_ACK);
	}else{
		pI2Cx->CR1 &= ~( 1 << I2C_CR1_ACK);
	}
}

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){
	if (EnorDi == ENABLE){
		if(pI2Cx == I2C1){
			I2C1_PCLK_EN();
		}else if(pI2Cx == I2C2){
			I2C2_PCLK_EN();
		}else if(pI2Cx == I2C3){
			I2C3_PCLK_EN();
		}
	}else{
		if(pI2Cx == I2C1){
			I2C1_PCLK_DI();
		}else if(pI2Cx == I2C2){
			I2C2_PCLK_DI();
		}else if(pI2Cx == I2C3){
			I2C3_PCLK_DI();
		}
	}
}

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle){
	if(pI2CHandle->TxLen > 0)
	{
		//1. load the data into DR
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

		//2. decrement the TxLen
		pI2CHandle->TxLen--;

		//3.Increment the buffer address
		pI2CHandle->pTxBuffer++;
	}
}

static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle){
	if(pI2CHandle->RxSize == 1){
				*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
				pI2CHandle->RxLen--;
			}

			if(pI2CHandle->RxSize > 1){
				if(pI2CHandle->RxLen == 2){
					// clear the ack bit
					I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
				}

				// read DR
				*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
				pI2CHandle->pRxBuffer++;
				pI2CHandle->RxLen--;
			}

			if(pI2CHandle->RxLen == 0){
				// close the I2C data reception and notify the application

				//1. generate the stop condition

				if(pI2CHandle->Sr == I2C_DISABLE_SR){
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
				}
				//2. Close the I2C Rx
				I2C_CloseReceiveData(pI2CHandle);

				//3.Notify the application
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
			}
}





void I2C_Init(I2C_Handle_t*pI2CHandle){
	uint32_t tempreg = 0;
	// enable the clock for the i2cx peripheral

	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	//ack control bit
	tempreg = pI2CHandle->I2C_Config.I2C_ACKControl << 10;
	pI2CHandle->pI2Cx->CR1 = tempreg;

	// configure the FREQ field of CR2

	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value() / 1000000U; // 16M / 1M = 16 cần số 16 này

	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F) ;

	// program the device own address
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	tempreg |= (1 << 14);
	pI2CHandle->pI2Cx->OAR1 = tempreg; // Cài đặt xong cho thanh ghi OAR1 về địa chỉ 10 bit hay 7 bit

	// tính toán CCR

	uint16_t ccr_value = 0;
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		// mode is standard mode
		ccr_value = RCC_GetPCLK1Value() / ( 2* pI2CHandle->I2C_Config.I2C_SCLSpeed);
		tempreg |= (ccr_value & 0xFFF);
	}else{
		// mode is fast mode
		tempreg |= ( 1 << 15); // Bật FM mode trên thanh ghi CCR
		tempreg |= ( pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14); // Chỉnh 1 trong hai chế độ của Fm Mode ở thanh ghi CCR
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2){
			ccr_value = RCC_GetPCLK1Value() / ( 3* pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}else{
			ccr_value = RCC_GetPCLK1Value() / ( 25* pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	//Trise Conifguration

	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		// mode is trandard mode

		tempreg = (RCC_GetPCLK1Value() / 1000000U) + 1;

	}else{
		// mode is fast mode
		tempreg = ((RCC_GetPCLK1Value() * 300) / 1000000000U ) + 1;
	}

	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);

}
void I2C_DeInint(I2C_RegDef_t *pI2Cx){

}

// Bật và tắt ngoại vi của I2C sử dụng bit 0 của thanh ghi I2C_CR1
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi){
	if( EnOrDi == ENABLE){
		pI2Cx->CR1 |= ( 1 << I2C_CR1_PE);
	}else{
		pI2Cx->CR1 &= ~( 1 << I2C_CR1_PE);
	}
}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName){
	if(pI2Cx->SR1 & FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}


void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr){
	//1. Generate the START condition

	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. confirm that start generation is completed by checking the SB flag in the SR1
	// Note: Until SB is cleared SCL will be stretched (pulled to LOW)

	while(I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_SB_FLAG));

	//3. Send the address of the slave with r/nw bit set to w(0) (total of 8bits)
	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, SlaveAddr);

	//4. Confirm that address phase is completed by checking the ADDR flag in the SR1

	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_ADDR_FLAG));

	//5. Clear the ADDR flag according to its software sequence
	// Note: Until ADDR is cleared SCL will be stretched (pulled to LOW)
	I2C_ClearADDRFlag(pI2CHandle);

	//6.send the data until Len becomes 0

	while(Len > 0){
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_TXE_FLAG)); // wait till TXE is set
		pI2CHandle->pI2Cx->DR = *pTxbuffer;
		Len--;
	}

	//7. When Len becomes zero wait for TXE=1 and BTF=1 before generating the STOP condition
	// Note: TXE=1, BTF=1, means that both SR and DR are empty and next transmission should begin
	// when BTF1 SCL will be stretched (pulled to LOW)
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_TXE_FLAG));

	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_BTF_FLAG));

	//8. Generate STOP condition and master need not to wait for the complettion of stop condition
	// Note: generating STOP, automatically clears the BTF

	if(Sr == I2C_DISABLE_SR){
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	}



}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr){
	//1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
	//2.Confirm that start generation is completed by checking the SB flag in the SR1
	// Note: Until SB is cleared SCL will be stretched (pulled to LOW)

	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SB_FLAG));

	//3. Send the address of the slave with r/nw bit set to R(1) (total 8 bits)

	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx,SlaveAddr);

	//4. wait until address phase is completed by checking the ADDR flag in the SR1

	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_ADDR_FLAG));

	// procedure to read only 1 byte from slave
	if ( Len == 1){
		// Disable Ack
		I2C_ManageAcking(pI2CHandle->pI2Cx,DISABLE);



		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//wait until RXNE becomes 1
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_TXE_FLAG));

		// generate the stop condition
		if(Sr == I2C_DISABLE_SR){
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}



		// read data in to buffer

		*pRxBuffer = pI2CHandle->pI2Cx->DR;

		return;
	}
		//procedure to read data from slave when Len > 1
			if(Len > 1)
			{
				//clear the ADDR flag
				I2C_ClearADDRFlag(pI2CHandle);

				//read the data until Len becomes zero
				for ( uint32_t i = Len ; i > 0 ; i--)
				{
					//wait until RXNE becomes 1
					while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_RXNE_FLAG) );

					if(i == 2) //if last 2 bytes are remaining
					{
						//Disable Acking
						I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_DISABLE);

						//generate STOP condition
						if(Sr == I2C_DISABLE_SR){
							I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
						}






				}
						//read the data from data register in to buffer
						*pRxBuffer = pI2CHandle->pI2Cx->DR;

						//increment the buffer address
						pRxBuffer++;

			}

			}

			if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE){
				// re-enable ACKING
				I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_ENABLE);
			}


}

/*********************************************************************
 * @fn      		  - I2C_MasterSendDataIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
uint8_t  I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len,uint8_t SlaveAddr,uint8_t Sr)
{

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);

	}

	return busystate;

}

/*********************************************************************
 * @fn      		  -
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr)
{

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
	}

	return busystate;
}

/*********************************************************************
 * @fn      		  - I2C_EV_IRQHandling
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle){
	// Interrupt handling for both master and slave mode of a device

	uint32_t temp1, temp2, temp3;

	temp1 = pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB);


	//1. Handle for interrupt generated by SB event
	// Note: SB flag is only applicable in Master mode
	if(temp1 && temp3){
			//SB flag is set
			//This work only in Master mode (RM)
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX){
			I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX){
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
			}
		}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);

	//2. Handle For interrupt generated by ADDR event
	//Note: When master mode : Address is sent
	// When Slave mode : address matched with own address
	if(temp1 && temp3){
		// ADDR flag is set
		// Have to clear the ADDR Flag

		I2C_ClearADDRFlag(pI2CHandle);
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);
	//3. Handle for interrupt generated by BTF (Byte transfer finished) event
	if (temp1 && temp3){
		//BTF flag is set
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX){
			// make sure that TXE is also set
			if(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TxE)){
				//BTF, TXE = 1
				if(pI2CHandle->TxLen == 0){


				//1. generatate the STOP condition
					if(pI2CHandle->Sr == I2C_DISABLE_SR)
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
				//2. reset all the member elements of the handle structure.
				I2C_CloseSendData(pI2CHandle);
				//3. notify the application about transmission complete
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
			}
		}else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX){

		}
		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF);
	//4. Handle For interrupt generated by STOPF event
	//Note: Stop detection flag is applicable only slave mode. For master this flag will
	if (temp1 && temp3){
			//STOPF flag is set
			//Clear the STOPF (i.e 1) read SR1 2) Write to CR1 )

		pI2CHandle->pI2Cx->CR1 |= 0x0000;

		//Notify the application that STOP is detected

		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TxE);
	//5. Handle For interrupt generated by TXE event
	if(temp1 && temp2 && temp3){

		if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL)){


		// Txe Flag is set
		// We have to do that data transmission

		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX){
			if(pI2CHandle->TxLen > 0)
			{
				//1. Load the data into DR
				pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

				//2. Decrement the TxLen
				pI2CHandle->TxLen--;
				//3.Increment the buffer address
				pI2CHandle->pTxBuffer++;



				}
			}
		}else{
			// for slave
			// make sure that the slave is really in transmitter mode
			if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA)){
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
			}


		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RxNE);
	//6. Handle for interrupt generated by RXNE event
	if(temp1 && temp2 && temp3){

		//check the device mode
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)){


		// RxNE flag set
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX){
			// We have to do the data reception
			I2C_MasterHandleRXNEInterrupt(pI2CHandle);

			}

		}else{
			// for slave mode
			//make sure that the slave is really in receiver mode
			if(!(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_TRA))){
				//slave mode
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
			}
		}
	}


}


void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE){
			I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
	}
}

void I2C_CLoseSendData(I2C_Handle_t *pI2CHandle){
	// Implement the code to disable ITBUFEN Control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
	//Implement the code to disable ITEVFEN Control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->TxLen = 0;

}

void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle){
	uint32_t temp1,temp2;

	    //Know the status of  ITERREN control bit in the CR2
		temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);


	/***********************Check for Bus error************************************/
		temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR);
		if(temp1  && temp2 )
		{
			//This is Bus error

			//Implement the code to clear the buss error flag
			pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

			//Implement the code to notify the application about the error
		   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
		}

	/***********************Check for arbitration lost error************************************/
		temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO );
		if(temp1  && temp2)
		{
			//This is arbitration lost error

			//Implement the code to clear the arbitration lost error flag
			pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_ARLO);
			//Implement the code to notify the application about the error
			I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_ARLO);
		}

	/***********************Check for ACK failure  error************************************/

		temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
		if(temp1  && temp2)
		{
			//This is ACK failure error

		    //Implement the code to clear the ACK failure error flag
			pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_AF);
			//Implement the code to notify the application about the error
			I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_AF);
		}

	/***********************Check for Overrun/underrun error************************************/
		temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
		if(temp1  && temp2)
		{
			//This is Overrun/underrun

		    //Implement the code to clear the Overrun/underrun error flag
			pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_OVR);
			//Implement the code to notify the application about the error
			I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_OVR);
		}

	/***********************Check for Time out error************************************/
		temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
		if(temp1  && temp2)
		{
			//This is Time out error

		    //Implement the code to clear the Time out error flag
			pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_TIMEOUT);
			//Implement the code to notify the application about the error
			I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_TIMEOUT);
		}

}

void I2C_SlaveSendData(I2C_RegDef_t *pI2C,uint8_t data){
	pI2C->DR = data;
}
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C){
	return (uint8_t) pI2C->DR;
}


