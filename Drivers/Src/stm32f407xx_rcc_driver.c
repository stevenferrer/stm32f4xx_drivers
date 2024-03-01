/*
 * stm32f407xx_rcc_driver.c
 *
 *  Created on: Mar 1, 2024
 *      Author: sf
 */

#include "stm32f407xx_rcc_driver.h"

uint16_t AHB_PreScaler[8] = { 2, 4, 8, 16, 64, 128, 256, 512 };
uint16_t APB_PreScaler[4] = { 2, 4, 8, 16 };

uint32_t RCC_GetPLLOutputClock(void) {
	// TODO: Implement
	return 0;
}

uint32_t RCC_GetPclk1Value(void) {
	// See reference manual for RCC->CFGR->SWS
	uint8_t clkSrc = (RCC->CFGR >> 2) & 0x3;

	uint32_t sysClk;
	if (clkSrc == 0) {
		// hsi
		sysClk = 16000000;
	} else if (clkSrc == 1) {
		// hse
		sysClk = 8000000;
	} else if (clkSrc == 2) { // pll
		sysClk = RCC_GetPLLOutputClock();
	}

	uint8_t ahbpReg = (RCC->CFGR >> 4) & 0xf;

	uint8_t ahbp = 1;
	if (ahbp < 8) {
		ahbp = 1;
	} else {
		ahbp = AHB_PreScaler[ahbpReg - 8];
	}

	uint8_t apb1pReg = (RCC->CFGR >> 10) & 0x7;

	uint8_t apb1p = 1;
	if (apb1pReg < 4) {
		apb1p = 1;
	} else {
		apb1p = APB_PreScaler[apb1pReg - 4];
	}

	uint32_t pclk1 = (sysClk / ahbp) / apb1p;

	return pclk1;
}

uint32_t RCC_GetPclk2Value(void) {
	return 0;
}
