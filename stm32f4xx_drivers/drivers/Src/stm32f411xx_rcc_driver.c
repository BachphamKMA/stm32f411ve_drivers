/*
 * stm32f411xx_rcc_driver.c
 *
 *  Created on: May 24, 2022
 *      Author: Bach
 */


#include "stm32f411xx_rcc_driver.h"



uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint8_t APB1_PreScaler[4] = {2,4,8,16};

// đoạn này để chỉnh Clock của AHB và ABP1 dùng cho I2C trả về tần số của PCLK1
uint32_t RCC_GetPCLK1Value(void){
	uint32_t pclk1,SystemClk;

	uint8_t clksrc, temp , ahbp, abp1p;
	clksrc = (RCC->CFGR >> 2) & 0x3; // các bit khác sẽ = 0 hết ngoại trừ bit 0 và bit 1


	if( clksrc == 0){
		SystemClk = 16000000;
	}else if(clksrc == 1){
		SystemClk = 8000000;
	}else if(clksrc == 2){
		SystemClk = RCC_GetPLLOutputClock();
	}

	temp = (RCC->CFGR >> 4) & 0xF;

	if(temp < 8 ){
		ahbp = 1;

	}else
	{
		ahbp = AHB_PreScaler[temp-8];
	}

	//apb1
	temp = ((RCC->CFGR >> 10) & 0x7);

	if(temp < 4){
		abp1p = 1 ;
	}else{
		abp1p = APB1_PreScaler[temp-4];
	}

	pclk1 = (SystemClk / ahbp) / abp1p;
	return pclk1;
}


uint32_t RCC_GetPCLK2Value(void)
{
	uint32_t SystemClock=0,tmp,pclk2;
	uint8_t clk_src = ( RCC->CFGR >> 2) & 0X3;

	uint8_t ahbp,apb2p;

	if(clk_src == 0)
	{
		SystemClock = 16000000;
	}else
	{
		SystemClock = 8000000;
	}
	tmp = (RCC->CFGR >> 4 ) & 0xF;

	if(tmp < 0x08)
	{
		ahbp = 1;
	}else
	{
       ahbp = AHB_PreScaler[tmp-8];
	}

	tmp = (RCC->CFGR >> 13 ) & 0x7;
	if(tmp < 0x04)
	{
		apb2p = 1;
	}else
	{
		apb2p = APB1_PreScaler[tmp-4];
	}

	pclk2 = (SystemClock / ahbp )/ apb2p;

	return pclk2;
}

uint32_t  RCC_GetPLLOutputClock()
{

	return 0;
}
