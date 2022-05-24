/*
 * stm32f411xx_spi_driver.c
 *
 *  Created on: May 3, 2022
 *      Author: Bach
 */


#include <stm32f411xx_spi_driver.h>

static void spi_txe_interrupt_handle(SPI_Handle_t *pHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pHandle);

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}else if (pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}else if (pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
	}
	else
	{
		if(pSPIx == SPI1){
			SPI1_PCLK_DI();
		}else if (pSPIx == SPI2){
			SPI2_PCLK_DI();
		}else if (pSPIx == SPI3){
			SPI3_PCLK_DI();
		}
	}
}


/*
 * Init và Deinit
 */
void SPI_Init(SPI_Handle_t*pSPIHandle){

	//peripheral clock enable

	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	// Config cho thanh ghi SPI_CR1


	uint32_t tempreg = 0;
	//1. configure the device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << 2;

	//2. configure the bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD){
		//BIDI mode should be cleared
		tempreg &= ~(1 << 15);
	}else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD){
		// bidi mode should be set
		tempreg |= 	(1 << 15);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_S_RXONLY){
		// bidi mode should be cleared
		tempreg &= ~(1 << 15);
		// RXONLY bit must be set
		tempreg |= (1 << 10);
	}


	// 3. configure the spi serial clock speed (baud rate)
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << 3;

	// 4. configure the DFF

	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << 11;

	// 5. configure the CPOL

	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << 1;


	// 6. configure the CPHA

	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	pSPIHandle->pSPIx->CR1 = tempreg;


}
void SPI_DeInint(SPI_RegDef_t *pSPIx){
	// TO DO
}

uint8_t SPI_Get_Flag_Status(SPI_RegDef_t *pSPIx, uint32_t FlagName){
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;

	}
	return FLAG_RESET;
}

/*
 * Data send and receive
 */

void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer, uint32_t Len){
	while (Len > 0){
		//1. wait until TXE is set
		while (SPI_Get_Flag_Status(pSPIx,SPI_TXE_FLAG) == FLAG_RESET);


		//2. Check the DFF bit in CR1
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
			//16 bit DFF
			//1. load the data in to the DR
			pSPIx->DR =  *((uint16_t*)pTxBuffer);
			Len --;
			Len --;
			(uint16_t*)pTxBuffer++;
		}else
		{
			//8 bit DFF
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}


void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer, uint32_t Len){
	while (Len > 0){
			//1. wait until TXE is set
			while (SPI_Get_Flag_Status(pSPIx,SPI_RXNE_FLAG) == FLAG_RESET);


			//2. Check the DFF bit in CR1
			if(pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
				//16 bit DFF
				//1. load the data from DR to Rxbuffer address
				*((uint16_t*)pRxBuffer) = pSPIx->DR;
				Len --;
				Len --;
				(uint16_t*)pRxBuffer++;
			}else
			{
				//8 bit DFF
				*(pRxBuffer) = pSPIx->DR;
				Len--;
				pRxBuffer++;
			}
		}
}

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);

	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){
	if(EnOrDi == ENABLE)
		{
			pSPIx->CR1 |= (1 << SPI_CR1_SSI);

		}else
		{
			pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
		}
}

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){
	if(EnOrDi == ENABLE)
		{
			pSPIx->CR2 |= (1 << SPI_CR1_SSI);

		}
	else
		{
			pSPIx->CR2 &= ~(1 << SPI_CR1_SSI);
		}
}


/*
 * Cài đặt và điều khiển SPI
 */

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){
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

	void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority){
			//1. first lets find out the ipr register
			uint8_t iprx = IRQNumber / 4;
			uint8_t iprx_section  = IRQNumber %4 ;

			uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

			*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );
	}





uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer, uint32_t Len){
	uint8_t state = pSPIHandle->TxState;

	if (state != SPI_BUSY_IN_TX){
		//1. Lưu địa chỉ của Tx buffer và độ dài Len vào biến cục bộ
			pSPIHandle->pTxBuffer = pTxBuffer;
			pSPIHandle->TxLen = Len;
			//2. Đánh dấu trạng thái SPI là bận để code khác không thể dùng ngoại vi của SPI đó và phải đợi đến khi việc truyền dữ liệu kết thúc
			pSPIHandle->TxState = SPI_BUSY_IN_TX;
			//3. Bật bit điều khiển TXEIE để thực hiện ngắt mỗi khi cờ TXE là 1 ở thanh ghi SR
			pSPIHandle->pSPIx->CR2 |=  (1 << SPI_CR2_TXEIE);

	}

	return state;

}
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pRxBuffer, uint32_t Len){
	uint8_t state = pSPIHandle->RxState;

	if (state != SPI_BUSY_IN_RX){
		//1. Lưu địa chỉ của Rx buffer và độ dài Len vào biến cục bộ
			pSPIHandle->pRxBuffer = pRxBuffer;
			pSPIHandle->RxLen = Len;
			//2. Đánh dấu trạng thái SPI là bận để code khác không thể dùng ngoại vi của SPI đó và phải đợi đến khi việc truyền dữ liệu kết thúc
			pSPIHandle->RxState = SPI_BUSY_IN_RX;
			//3. Bật bit điều khiển RXNEIE để thực hiện ngắt mỗi khi cờ RXE là 1 ở thanh ghi SR
			pSPIHandle->pSPIx->CR2 |=  (1 << SPI_CR2_RXNEIE);

}
	return state;
}



void SPI_IRQHandling(SPI_Handle_t *pHandle){
	uint8_t temp1, temp2;
	// first let check for TXE
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_TXE); // nếu bit TXE = 1 thì temp 1 sẽ nhận giá trị 1 và ngược lại ( vì thanh ghi SR mà nhân với 0000..1000 thì sẽ trả về một kết quả dương nên trong điều kiện if sẽ hiểu là 1)
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_TXEIE); // nếu bit TXEIE = 1 thì temp 2 sẽ nhận giá trị 1 và ngược lại

	// Kiểm tra nếu temp1, temp2 đồng thời = 1 ( true) thì sẽ thực hiện hàm bên dưới ( handle TXE)
	if(temp1 && temp2){
		//handle TXE
		spi_txe_interrupt_handle(pHandle);

	}

	// kiểm tra RXNE
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_RXNEIE);


	if(temp1 && temp2){
			//handle RXNE
			spi_rxne_interrupt_handle(pHandle);

	}
	// kiểm tra cờ lỗi cờ tràn (OVR)
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_ERRIE);

	if(temp1 && temp2){
		// handle ovr error
		spi_ovr_err_interrupt_handle(pHandle);
	}

}

static void spi_txe_interrupt_handle(SPI_Handle_t *pHandle){
	//2. Check the DFF bit in CR1
		if(pHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
			//16 bit DFF
			//1. load the data in to the DR
			pHandle->pSPIx->DR =  *((uint16_t*)pHandle->pTxBuffer);
			pHandle->TxLen--;
			pHandle->TxLen--;
			(uint16_t*)pHandle->pTxBuffer++;
		}else
		{
			//8 bit DFF
			pHandle->pSPIx->DR = *pHandle->pTxBuffer;
			pHandle->TxLen--;
			pHandle->pTxBuffer++;
		}

		if(! pHandle->TxLen){
			// nếu Txlen = - thì đống giao tiếp spi và phải thông báo cho application là Tx đã hết
			// ngăn chặn ngắt từ cài đặt của cờ TXE
			pHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_TXEIE);
			pHandle->pTxBuffer = NULL;
			pHandle->TxLen = 0;
			pHandle->TxState = SPI_READY;
			SPI_ApplicationEventCallback(pHandle,SPI_EVENT_TX_CMPLT);
		}
}
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle){
		//do rxing as per the dff
		if(pSPIHandle->pSPIx->CR1 & ( 1 << 11))
		{
			//16 bit
			*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t) pSPIHandle->pSPIx->DR;
			pSPIHandle->RxLen -= 2;
			pSPIHandle->pRxBuffer++;
			pSPIHandle->pRxBuffer++;

		}else
		{
			//8 bit
			*(pSPIHandle->pRxBuffer) = (uint8_t) pSPIHandle->pSPIx->DR;
			pSPIHandle->RxLen--;
			pSPIHandle->pRxBuffer++;
		}

		if(! pSPIHandle->RxLen)
		{
			//reception is complete
			SPI_CloseReception(pSPIHandle);
			SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);
		}
}
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pHandle){
	// 1. Clear the ovr flag

	uint8_t temp;
	if(pHandle->TxState != SPI_BUSY_IN_TX){
		temp = pHandle->pSPIx->DR;
		temp = pHandle->pSPIx->SR;
	}
	(void)temp;
	//2. Inform the application
	SPI_ApplicationEventCallback(pHandle,SPI_EVENT_TX_CMPLT);


}

void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle){
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}
void SPI_CloseReception(SPI_Handle_t *pSPIHandle){
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;

}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx){
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}

__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{

	//This is a weak implementation . the user application may override this function.
}
