/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: Feb 5, 2024
 *      Author: sf
 */

#include "stm32f407xx.h"
#include "stm32f407xx_i2c_driver.h"

uint16_t AHB_PreScaler[8] = { 2, 4, 8, 16, 64, 128, 256, 512 };
uint16_t APB_PreScaler[4] = { 2, 4, 8, 16 };

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecAddrPhase(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr);
static void I2C_ClearAddrFlag(I2C_RegDef_t *pI2Cx);

/*
 * Peripheral clock setup
 */
void I2C_PeriClockCtrl(I2C_RegDef_t *pI2Cx, uint8_t enable) {
	if (enable == ENABLE) {
		if (pI2Cx == I2C1) {
			I2C1_PCLK_EN();
		} else if (pI2Cx == I2C2) {
			I2C2_PCLK_EN();
		} else if (pI2Cx == I2C3) {
			I2C3_PCLK_EN();
		}
	} else {
		if (pI2Cx == I2C1) {
			I2C1_PCLK_DI();
		} else if (pI2Cx == I2C2) {
			I2C2_PCLK_DI();
		} else if (pI2Cx == I2C3) {
			I2C3_PCLK_DI();
		}
	}
}

uint32_t RCC_GetPLLOutputClock(void) {
	// TODO: Implement
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

/*
 * Init and de-init
 */
void I2C_Init(I2C_Handle_t *pI2CHandle) {
	// Note: Configurations must be done while peripheral is disabled in the control register
	// 1. Configure mode (std or fast)
	// 2. Configure speed of serial clock
	// 3. Configure the device adddress (when device is slave)
	// 4. Enable ack
	// 5. Configure the rise time for I2C pins

	// configure ack control bit of CR1
	uint32_t tempReg;
	tempReg |= pI2CHandle->config.ACKControl << I2C_CR1_ACK;
	pI2CHandle->i2cx->CR1 |= tempReg;

	// configure freq field of CR2
	// divide by 1MHz to get smaller value e.g. 16
	tempReg = RCC_GetPclk1Value() / 1000000u;
	pI2CHandle->i2cx->CR2 |= (tempReg & 0x3f);

	// TODO: Add support for 10-bit slave addressing (I2C_OAR1->ADD_MODE)
	// program device own address
	tempReg = pI2CHandle->config.DeviceAddress << 1;
	// as per reference manual, bit 14 of I2C->OAR1
	// should be kept at 1 by software \_('_')_/
	tempReg |= (1 << 14);
	pI2CHandle->i2cx->OAR1 |= tempReg;

	// configure CCR, see reference manual for formulas
	uint16_t ccrValue = 0;
	if (pI2CHandle->config.SCLSpeed <= I2C_SCL_SPEED_STD) {
		// standard mode
		ccrValue = RCC_GetPclk1Value() / (2 * pI2CHandle->config.SCLSpeed);
		tempReg = ccrValue & 0xfff;
	} else {
		// fast mode
		tempReg = (1 << 15); // enable fast mode
		tempReg |= (pI2CHandle->config.FMDutyCycle << 14);

		if (pI2CHandle->config.FMDutyCycle == I2C_FM_DUTY_2) {
			ccrValue = RCC_GetPclk1Value() / (3 * pI2CHandle->config.SCLSpeed);
		} else {
			ccrValue = RCC_GetPclk1Value() / (25 * pI2CHandle->config.SCLSpeed);
		}

		tempReg = ccrValue & 0xfff;
	}

	pI2CHandle->i2cx->CCR |= tempReg;

	// configure trise

}

void I2C_DeInit(I2C_RegDef_t *pI2Cx) {
	// reset register
}

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer,
		uint32_t len, uint8_t slaveAddr) {
	// 1. Generate start condition
	I2C_GenerateStartCondition(pI2CHandle->i2cx);

	// 2. Confirm that start generation is complete by checking the SB flag in the SR1.
	// Note: Until SB is cleared, SCL will be stretched (pulled to low)
	while (!I2C_GetFlagStatus(pI2CHandle->i2cx, I2C_FLAG_SB))
		;

	// 3. Send the address of the slave with r/w bit set to w(0) (total 8 bits).
	I2C_ExecAddrPhase(pI2CHandle->i2cx, slaveAddr);

	// 4. Confirm that address phase is completed by checking the ADDR flag in the SR1.
	while (!I2C_GetFlagStatus(pI2CHandle->i2cx, I2C_FLAG_ADDR))
		;

	// 5. Clear the ADDR flag according to its software sequence.
	// Note: Until the ADDR is cleared, SCL will be stretched (pulled to low)
	I2C_ClearAddrFlag(pI2CHandle->i2cx);

	// 6. Send the data until len becomes 0.
	while (len > 0) {
		// wait till txe is set
		while (!I2C_GetFlagStatus(pI2CHandle->i2cx, I2C_FLAG_TXE))
			;
		pI2CHandle->i2cx->DR = *pTxBuffer;
		pTxBuffer++;
		len--;
	}

	// 7. When len becomes zero, wait for TXE=1 and BTF=1 before generating the STOP condition.
	// Note: TXE=1, BTF=1, means that both SR and DR are empty and next transmission should begin
	// when BTF=1, SCL will be stretched (pulled to low)
	while (!I2C_GetFlagStatus(pI2CHandle->i2cx, I2C_FLAG_TXE))
		;
	while (!I2C_GetFlagStatus(pI2CHandle->i2cx, I2C_FLAG_BTF))
		;

	// 8. Generate STOP condition and master need not to wait for the completion of stop condition.
	// Note: Generating STOP automatically clears the BTF.
	I2C_GenerateStopCondition(pI2CHandle->i2cx);
}

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx) {
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx) {
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

static void I2C_ExecAddrPhase(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr) {
	// send 7-bit address

	// make space for r/w bit
	slaveAddr = slaveAddr << 1;
	// clear first bit
	slaveAddr &= !(1);
	// set slave addr to DR
	pI2Cx->DR = slaveAddr;
}

static void I2C_ClearAddrFlag(I2C_RegDef_t *pI2Cx) {
	uint32_t dummyRead = pI2Cx->SR1;
	dummyRead = pI2Cx->SR1;
	(void) dummyRead;
}

/*
 * IRQ config and ISR handling
 */
void I2C_IRQInterruptConfig(uint8_t irqNumber, uint8_t enable) {
	if (enable == ENABLE) {
		// program ISER register
		if (irqNumber <= 31) {
			// ISER0
			*NVIC_ISER0 |= (1 << irqNumber);
		} else if (irqNumber > 31 && irqNumber < 64) {
			// ISER1
			*NVIC_ISER1 |= (1 << (irqNumber % 32));
		} else if (irqNumber > 64 && irqNumber < 96) {
			// ISER2
			*NVIC_ISER2 |= (1 << (irqNumber % 64));
		}

	} else {
		// program ICER register
		if (irqNumber <= 31) {
			// ICER0
			*NVIC_ICER0 |= (1 << irqNumber);
		} else if (irqNumber > 31 && irqNumber < 64) {
			// ICER1
			*NVIC_ICER1 |= (1 << irqNumber % 32);
		} else if (irqNumber > 64 && irqNumber < 96) {
			// ICER2
			*NVIC_ICER2 |= (1 << irqNumber % 64);
		}
	}
}

void I2C_IRQPriorityConfig(uint8_t irqNumber, uint8_t irqPriority) {
	// 1. find out the IPR register
	// example IRQ number is 236:
	// * 23 / 4 = 5 -> IPR 5
	// * 23 % 4 = 3 -> section 3rd
	uint8_t iprxAddrOffset = (irqNumber / 4);
	uint8_t iprxSection = irqNumber % 4;

	uint8_t shiftAmount = (8 * iprxSection)
			+ (8 - NO_PRIORITY_BITS_IMPLEMENTED);
	_reg *nvicIprPtr = (NVIC_IPR_BASE_ADDR + iprxAddrOffset);
	*nvicIprPtr |= (irqPriority << shiftAmount);
}

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t enable) {
	if (enable == ENABLE) {
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	} else {
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t mask) {
	if (pI2Cx->SR1 & mask) {
		return FLAG_SET;
	}
	return FLAG_RESET;
}

_weak void I2C_AppEventCallback(I2C_Handle_t *pI2CHandle, uint8_t event) {
}
