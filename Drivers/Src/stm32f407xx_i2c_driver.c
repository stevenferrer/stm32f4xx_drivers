/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: Feb 5, 2024
 *      Author: sf
 */

#include "stm32f407xx.h"
#include "stm32f407xx_i2c_driver.h"

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

/*
 * Init and de-init
 */
void I2C_Init(I2C_Handle_t *pI2CHandle) {
	// Note: Must be done while peripheral is disabled in the control register
	// 1. Configure mode (std or fast)
	// 2. Configure speed of serial clock
	// 3. Configure the device adddress (when device is slave)
	// 4. Enable ack
	// 5. Configure the rise time for I2C pins
}

void I2C_DeInit(I2C_RegDef_t *pI2Cx) {
	// reset register
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

uint8_t I2C_GetStatusFlag(I2C_RegDef_t *pI2Cx, uint32_t mask) {
}

_weak void I2C_AppEventCallback(I2C_Handle_t *pI2CHandle, uint8_t event) {
}

