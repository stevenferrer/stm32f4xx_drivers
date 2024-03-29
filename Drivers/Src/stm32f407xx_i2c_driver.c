/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: Feb 5, 2024
 *      Author: sf
 */

#include <stdio.h>

#include "stm32f407xx.h"
#include "stm32f407xx_i2c_driver.h"
#include "stm32f407xx_rcc_driver.h"

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecAddrPhase(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr,
		uint8_t enable);
static void I2C_ClearAddrFlag(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleTxeInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleRxneInterrupt(I2C_Handle_t *pI2CHandle);

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
	// Note: Configurations must be done while peripheral is disabled in the control register
	// 1. Configure mode (std or fast)
	// 2. Configure speed of serial clock
	// 3. Configure the device adddress (when device is slave)
	// 4. Enable ack
	// 5. Configure the rise time for I2C pins

	// enable peripheral clock
	I2C_PeriClockCtrl(pI2CHandle->i2cx, ENABLE);

	// configure ack control bit of CR1
	uint32_t tempReg;

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
	if (pI2CHandle->config.SCLSpeed <= I2C_SCL_SPEED_STD) {
		// standard mode
		tempReg = (RCC_GetPclk1Value() / 1000000U) + 1;
	} else {
		// fast mode
		tempReg = (RCC_GetPclk1Value() * 300 / 1000000000U) + 1;
	}

	pI2CHandle->i2cx->TRISE |= tempReg & 0x3f;

}

void I2C_DeInit(I2C_RegDef_t *pI2Cx) {
	// reset register
}

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer,
		uint32_t len, uint8_t slaveAddr, uint8_t sr) {
	// 1. Generate start condition
	I2C_GenerateStartCondition(pI2CHandle->i2cx);

	// 2. Confirm that start generation is complete by checking the SB flag in the SR1.
	// Note: Until SB is cleared, SCL will be stretched (pulled to low)
	while (!I2C_GetFlagStatus(pI2CHandle->i2cx, I2C_FLAG_SB))
		;

	// 3. Send the address of the slave with r/w bit set to w(0) (total 8 bits).
	I2C_ExecAddrPhase(pI2CHandle->i2cx, slaveAddr, I2C_RW_WRITE);

	// 4. Confirm that address phase is completed by checking the ADDR flag in the SR1.
	while (!I2C_GetFlagStatus(pI2CHandle->i2cx, I2C_FLAG_ADDR))
		;

	// 5. Clear the ADDR flag according to its software sequence.
	// Note: Until the ADDR is cleared, SCL will be stretched (pulled to low)
	I2C_ClearAddrFlag(pI2CHandle);

	// 6. Send the data until len becomes 0.
	while (len > 0) {
		// wait till txe is set
		if (!I2C_GetFlagStatus(pI2CHandle->i2cx, I2C_FLAG_TXE))
			continue;

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
	if (sr != I2C_SR_ENABLE) {
		I2C_GenerateStopCondition(pI2CHandle->i2cx);
	}
}

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx) {
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx) {
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

static void I2C_ExecAddrPhase(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr,
		uint8_t rw) {
	// send 7-bit address

	// make space for r/w bit
	slaveAddr = slaveAddr << 1;

	if (rw == I2C_RW_WRITE) {
		// clear first bit (rw=0)
		slaveAddr &= ~(1);
	} else {
		// set first bit (rw=1)
		slaveAddr |= 1;
	}

	// set slave addr to DR
	pI2Cx->DR = slaveAddr;
}

static void I2C_ClearAddrFlag(I2C_Handle_t *pI2CHandle) {
	uint32_t dummyRead;

	// check device mode
	if (pI2CHandle->i2cx->SR2 & (1 << I2C_SR2_MSL)) {
		// master mode

		if (pI2CHandle->state == I2C_STATE_BUSY_IN_RX) {
			if (pI2CHandle->rxSize == 1) {
				I2C_EnableAcking(pI2CHandle->i2cx, I2C_ACK_DISABLE);

				// clear addr flag by reading sr1 and sr2
				dummyRead = pI2CHandle->i2cx->SR1;
				dummyRead = pI2CHandle->i2cx->SR2;
				(void) dummyRead;
			}
		} else {
			// clear addr flag by reading sr1 and sr2
			dummyRead = pI2CHandle->i2cx->SR1;
			dummyRead = pI2CHandle->i2cx->SR2;
			(void) dummyRead;
		}

	} else {
		// slave mode

		// clear addr flag by reading sr1 and sr2

		dummyRead = pI2CHandle->i2cx->SR1;
		dummyRead = pI2CHandle->i2cx->SR2;
		(void) dummyRead;
	}
}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer,
		uint32_t len, uint8_t slaveAddr, uint8_t sr) {
	// 1. Generate teh start condition
	// 2. Confirm that start generation is complete by checking the SB flag in the SR1
	// Note: Until SB is cleared SCL will be stretched (pulled to low)

	// 1. Generate start condition
	I2C_GenerateStartCondition(pI2CHandle->i2cx);

	// 2. Confirm that start generation is complete by checking the SB flag in the SR1.
	// Note: Until SB is cleared, SCL will be stretched (pulled to low)
	while (!I2C_GetFlagStatus(pI2CHandle->i2cx, I2C_FLAG_SB))
		;

	// 3. Send the address of the slave with r/w bit set to R(1) (total of 8 bits)
	I2C_ExecAddrPhase(pI2CHandle->i2cx, slaveAddr, I2C_RW_READ);

	// 4. Confirm that address phase is completed by checking the ADDR flag in the SR1.
	while (!I2C_GetFlagStatus(pI2CHandle->i2cx, I2C_FLAG_ADDR))
		;

	// procedure to read only 1 byte from slave
	if (len == 1) {
		// disable acking
		I2C_EnableAcking(pI2CHandle->i2cx, I2C_ACK_DISABLE);

		// clear ADDR flag
		I2C_ClearAddrFlag(pI2CHandle);

		// wait until RXNE becomes 1
		while (!I2C_GetFlagStatus(pI2CHandle->i2cx, I2C_FLAG_RXNE))
			;

		// generate STOP condition
		if (sr != I2C_SR_ENABLE) {
			I2C_GenerateStopCondition(pI2CHandle->i2cx);
		}

		// read data into buffer
		*pRxBuffer = pI2CHandle->i2cx->DR;
	} else {
		// read multiple bytes from slave

		// clear addr flag
		I2C_ClearAddrFlag(pI2CHandle);

		// read data until len becomes zero
		for (uint32_t i = len; i > 0; --i) {
			// wait until RXNE becomes 1
			while (!I2C_GetFlagStatus(pI2CHandle->i2cx, I2C_FLAG_RXNE))
				;

			// read last 2 bytes
			if (i == 2) {
				// disable acking
				I2C_EnableAcking(pI2CHandle->i2cx, I2C_ACK_DISABLE);

				// generate STOP condition
				if (sr != I2C_SR_ENABLE) {
					I2C_GenerateStopCondition(pI2CHandle->i2cx);
				}
			}

			// read data into buffer
			*pRxBuffer = pI2CHandle->i2cx->DR;
			pRxBuffer++;
		}
	}

	// re-enable acking
	if (pI2CHandle->config.ACKControl == I2C_ACK_ENABLE) {
		I2C_EnableAcking(pI2CHandle->i2cx, I2C_ACK_ENABLE);
	}
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

void I2C_EnableAcking(I2C_RegDef_t *pI2Cx, uint8_t enable) {
	if (enable == I2C_ACK_ENABLE) {
		// enable ack
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	} else {
		// disable ack
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}
}

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer,
		uint32_t len, uint8_t slaveAddress, uint8_t sr) {
	uint8_t busyState = pI2CHandle->state;

	if ((busyState != I2C_STATE_BUSY_IN_TX)
			&& (busyState != I2C_STATE_BUSY_IN_RX)) {
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->txLen = len;
		pI2CHandle->state = I2C_STATE_BUSY_IN_TX;
		pI2CHandle->devAddr = slaveAddress;
		pI2CHandle->sr = sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->i2cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->i2cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->i2cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->i2cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}

	return busyState;
}

uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer,
		uint32_t len, uint8_t slaveAddress, uint8_t sr) {
	uint8_t busyState = pI2CHandle->state;

	if ((busyState != I2C_STATE_BUSY_IN_TX)
			&& (busyState != I2C_STATE_BUSY_IN_RX)) {
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->rxLen = len;
		pI2CHandle->rxSize = len;
		pI2CHandle->state = I2C_STATE_BUSY_IN_RX;
		pI2CHandle->devAddr = slaveAddress;
		pI2CHandle->sr = sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->i2cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->i2cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->i2cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->i2cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}

	return busyState;
}

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle) {
	//Interrupt handling for both master and slave mode of a device

	uint32_t itEvtEnFlag, itBufEnFlag, sr1Flag;

	itEvtEnFlag = pI2CHandle->i2cx->CR2 & (1 << I2C_CR2_ITEVTEN);
	itBufEnFlag = pI2CHandle->i2cx->CR2 & (1 << I2C_CR2_ITBUFEN);

	//1. Handle For interrupt generated by SB event
	//	Note : SB flag is only applicable in Master mode
	sr1Flag = pI2CHandle->i2cx->SR1 & (1 << I2C_SR1_SB);
	if (itEvtEnFlag && sr1Flag) {
		printf("SB event");
		// This interrupt is generated bc of SB event.
		// Only applicable in master mode.

		if (pI2CHandle->state == I2C_STATE_BUSY_IN_TX) {
			I2C_ExecAddrPhase(pI2CHandle->i2cx, pI2CHandle->devAddr,
			I2C_RW_WRITE);
		} else if (pI2CHandle->state == I2C_STATE_BUSY_IN_RX) {
			I2C_ExecAddrPhase(pI2CHandle->i2cx, pI2CHandle->devAddr,
			I2C_RW_READ);
		}
	}

	//2. Handle For interrupt generated by ADDR event
	//Note: When master mode: Address is sent
	//		When Slave mode: Address matched with own address
	sr1Flag = pI2CHandle->i2cx->SR1 & (1 << I2C_SR1_ADDR);
	if (itEvtEnFlag && sr1Flag) {
		printf("ADDR event");
		// This interrupt is generated bc of ADDR event
		I2C_ClearAddrFlag(pI2CHandle);
	}

	//3. Handle For interrupt generated by BTF(Byte Transfer Finished) event
	sr1Flag = pI2CHandle->i2cx->SR1 & (1 << I2C_SR1_BTF);
	if (itEvtEnFlag && sr1Flag) {
		printf("BTF event");
		// BTF flag is set
		if (pI2CHandle->state == I2C_STATE_BUSY_IN_TX) {
			// Make sure TXE is set
			if (pI2CHandle->i2cx->SR1 & (1 << I2C_SR1_TXE)) {
				// BTF, TXE = 1

				if (pI2CHandle->txLen == 0) {
					// 1. Generate stop condition
					if (pI2CHandle->sr == I2C_SR_DISABLE) {
						I2C_GenerateStopCondition(pI2CHandle->i2cx);
					}
					// 2. Reset all member elements of handle
					I2C_CloseSendData(pI2CHandle);	// maybe _Reset

					// 3. Notify the application
					I2C_AppEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
				}

			}

		} else if (pI2CHandle->state == I2C_STATE_BUSY_IN_RX) {
			// nothing to do here
		}
	}

	//4. Handle For interrupt generated by STOPF event
	// Note : Stop detection flag is applicable only slave mode. For master this flag will never be set.
	sr1Flag = pI2CHandle->i2cx->SR1 & (1 << I2C_SR1_STOPF);
	if (itEvtEnFlag && sr1Flag) {
		printf("STOPFF event");

		// STOPF flag is set
		// Clear the STOPF flag by 1. reading from SR1, 2. write to CR1
		pI2CHandle->i2cx->CR1 |= 0x0000;

		// Notify the application
		I2C_AppEventCallback(pI2CHandle, I2C_EV_STOP);
	}

	//5. Handle For interrupt generated by TXE event
	sr1Flag = pI2CHandle->i2cx->SR1 & (1 << I2C_SR1_TXE);
	if (itEvtEnFlag && itBufEnFlag && sr1Flag) {
		printf("TXE event");
		// TXE flag is set

		// check device mode (master/slave)
		if (pI2CHandle->i2cx->SR2 && (1 << I2C_SR2_MSL)) {
			if (pI2CHandle->state == I2C_STATE_BUSY_IN_TX) {
				I2C_MasterHandleTxeInterrupt(pI2CHandle);
			}
		}

	}

	//6. Handle For interrupt generated by RXNE event
	sr1Flag = pI2CHandle->i2cx->SR1 & (1 << I2C_SR1_RXNE);
	if (itEvtEnFlag && itBufEnFlag && sr1Flag) {
		printf("RXNE event");
		// RXNE flag is set

		// check device mode
		if (pI2CHandle->i2cx->SR2 && (1 << I2C_SR2_MSL)) {
			if (pI2CHandle->state == I2C_STATE_BUSY_IN_RX) {
				I2C_MasterHandleRxneInterrupt(pI2CHandle);
			}
		}
	}
}

void I2C_MasterHandleTxeInterrupt(I2C_Handle_t *pI2CHandle) {
	if (pI2CHandle->txLen > 0) {
		// transmit 1 byte of data
		pI2CHandle->i2cx->DR = *(pI2CHandle->pTxBuffer);
		pI2CHandle->pTxBuffer++;
		pI2CHandle->txLen--;
	}
}

void I2C_MasterHandleRxneInterrupt(I2C_Handle_t *pI2CHandle) {
	if (pI2CHandle->rxSize == 1) {
		*pI2CHandle->pRxBuffer = pI2CHandle->i2cx->DR;
		pI2CHandle->rxLen--;
	}

	if (pI2CHandle->rxSize > 1) {
		if (pI2CHandle->rxLen == 2) {
			I2C_EnableAcking(pI2CHandle->i2cx, I2C_ACK_DISABLE);
		}

		*pI2CHandle->pRxBuffer = pI2CHandle->i2cx->DR;
		pI2CHandle->pRxBuffer++;
		pI2CHandle->rxLen--;
	}

	if (pI2CHandle->rxLen == 0) {
		// close rx and notify application

		// generate stop condition
		if (pI2CHandle->sr == I2C_SR_DISABLE) {
			I2C_GenerateStartCondition(pI2CHandle->i2cx);
		}

		// close rx
		I2C_CloseReceiveData(pI2CHandle);

		// notify application
		I2C_AppEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
	}
}

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle) {
	// disable ITBUFEN
	pI2CHandle->i2cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	// disable ITEVTEN
	pI2CHandle->i2cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	// reset handle
	pI2CHandle->state = I2C_STATE_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->rxLen = 0;
	pI2CHandle->rxSize = 0;

	// enable acking
	if (pI2CHandle->config.ACKControl == I2C_ACK_ENABLE) {
		I2C_EnableAcking(pI2CHandle->i2cx, I2C_ACK_ENABLE);
	}
}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle) {
	// disable ITBUFEN
	pI2CHandle->i2cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	// disable ITEVTEN
	pI2CHandle->i2cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	// reset handle
	pI2CHandle->state = I2C_STATE_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->txLen = 0;

}

void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle) {
	uint32_t itErrEnFlag, sr1Flag;

	//Know the status of  ITERREN control bit in the CR2
	itErrEnFlag = (pI2CHandle->i2cx->CR2) & (1 << I2C_CR2_ITERREN);

	// bus error
	sr1Flag = (pI2CHandle->i2cx->SR1) & (1 << I2C_SR1_BERR);
	if (itErrEnFlag && sr1Flag) {
		//This is Bus error

		//Implement the code to clear the buss error flag
		pI2CHandle->i2cx->SR1 &= ~(1 << I2C_SR1_BERR);

		//Implement the code to notify the application about the error
		I2C_AppEventCallback(pI2CHandle, I2C_ERROR_BERR);
	}

	// arbitration lost error
	sr1Flag = (pI2CHandle->i2cx->SR1) & (1 << I2C_SR1_ARLO);
	if (itErrEnFlag && sr1Flag) {
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->i2cx->SR1 &= ~(1 << I2C_SR1_ARLO);

		//Implement the code to notify the application about the error
		I2C_AppEventCallback(pI2CHandle, I2C_ERROR_ARLO);
	}

	// ack failure error
	sr1Flag = (pI2CHandle->i2cx->SR1) & (1 << I2C_SR1_AF);
	if (itErrEnFlag && sr1Flag) {
		//This is ACK failure error

		//Implement the code to clear the ACK failure error flag
		pI2CHandle->i2cx->SR1 &= ~(1 << I2C_SR1_AF);

		//Implement the code to notify the application about the error
		I2C_AppEventCallback(pI2CHandle, I2C_ERROR_AF);
	}

	// overrun/underran error
	sr1Flag = (pI2CHandle->i2cx->SR1) & (1 << I2C_SR1_OVR);
	if (itErrEnFlag && sr1Flag) {
		//This is Overrun/underrun

		//Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->i2cx->SR1 &= ~(1 << I2C_SR1_OVR);

		//Implement the code to notify the application about the error
		I2C_AppEventCallback(pI2CHandle, I2C_ERROR_OVR);
	}

	// timeout error
	sr1Flag = (pI2CHandle->i2cx->SR1) & (1 << I2C_SR1_TIMEOUT);
	if (itErrEnFlag && sr1Flag) {
		//This is Time out error

		//Implement the code to clear the Time out error flag
		pI2CHandle->i2cx->SR1 &= ~(1 << I2C_SR1_TIMEOUT);

		//Implement the code to notify the application about the error
		I2C_AppEventCallback(pI2CHandle, I2C_ERROR_TIMEOUT);
	}

}

_weak void I2C_AppEventCallback(I2C_Handle_t *pI2CHandle, uint8_t event) {
}

