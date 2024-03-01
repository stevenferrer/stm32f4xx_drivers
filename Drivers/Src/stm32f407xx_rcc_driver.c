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

	uint8_t ahbp, apb1p;
	uint8_t clkSrc = (RCC->CFGR >> 2) & 0x3;

	// clock source
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

	// ahb prescaler
	uint8_t ahbpPos = (RCC->CFGR >> 4) & 0xf;
	if (ahbpPos < 8) {
		ahbp = 1;
	} else {
		ahbp = AHB_PreScaler[ahbpPos - 8];
	}

	// apb1 prescaler
	uint8_t apb1pPos = (RCC->CFGR >> 10) & 0x7;
	if (apb1pPos < 4) {
		apb1p = 1;
	} else {
		apb1p = APB_PreScaler[apb1pPos - 4];
	}

	uint32_t pclk1 = (sysClk / ahbp) / apb1p;

	return pclk1;
}

uint32_t RCC_GetPclk2Value(void) {
	uint32_t sysClk = 0, pclk2;

	uint8_t ahbp, apb2p;

	uint8_t clkSrc = ( RCC->CFGR >> 2) & 0X3;
	if (clkSrc == 0) {
		sysClk = 16000000;
	} else {
		sysClk = 8000000;
	}

	uint32_t ahbpPos = (RCC->CFGR >> 4) & 0xF;
	if (ahbpPos < 0x08) {
		ahbp = 1;
	} else {
		ahbp = AHB_PreScaler[ahbpPos - 8];
	}

	uint32_t apb2pPos = (RCC->CFGR >> 13) & 0x7;
	if (apb2pPos < 0x04) {
		apb2p = 1;
	} else {
		apb2p = APB_PreScaler[apb2pPos - 4];
	}

	pclk2 = (sysClk / ahbp) / apb2p;

	return pclk2;
}
