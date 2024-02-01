/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Jan 18, 2024
 *      Author: sf
 */

#include <stdlib.h>

#include "stm32f407xx_spi_driver.h"

// static functions are local the file where it was defined
static void SPI_TXE_IT_Handle(SPI_Handle_t *pSPIHandle);
static void SPI_RXNE_IT_Handle(SPI_Handle_t *pSPIHandle);
static void SPI_OVR_ERR_IT_Handle(SPI_Handle_t *pSPIHandle);

/*
 * Peripheral clock setup
 */
void SPI_PeriClockCtrl(SPI_RegDef_t *pSPIx, uint8_t enable) {
	if (enable == ENABLE) {
		if (pSPIx == SPI1) {
			SPI1_PCLK_EN();
		} else if (pSPIx == SPI2) {
			SPI2_PCLK_EN();
		} else if (pSPIx == SPI3) {
			SPI3_PCLK_EN();
		}
	} else {
		if (pSPIx == SPI1) {
			SPI1_PCLK_DI();
		} else if (pSPIx == SPI2) {
			SPI2_PCLK_DI();
		} else if (pSPIx == SPI3) {
			SPI3_PCLK_DI();
		}
	}

}

/*
 * Init and de-init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle) {
	// 1. Configure SPI_CR1 register
	uint32_t tempReg = 0;

	SPI_PeriClockCtrl(pSPIHandle->pSPIx, ENABLE);

	tempReg |= pSPIHandle->config.DeviceMode << SPI_CR1_MSTR; // MSTR

	if (pSPIHandle->config.BusConfig == SPI_BUS_CONFIG_FD) {
		// clear BIDIMODE
		tempReg &= ~(1 << SPI_CR1_BIDIMODE);
	} else if (pSPIHandle->config.BusConfig == SPI_BUS_CONFIG_HD) {
		// set BIDIMODE
		tempReg |= (1 << SPI_CR1_BIDIMODE);
	} else if (pSPIHandle->config.BusConfig == SPI_BUS_CONFIG_SIMPLEX_RX_ONLY) {
		// clear BIDIMODE
		tempReg &= ~(1 << SPI_CR1_BIDIMODE);
		// set RXONLY
		tempReg |= (1 << SPI_CR1_RXONLY);
	}

	tempReg |= (pSPIHandle->config.SCLKSpeed << SPI_CR1_BR);
	tempReg |= (pSPIHandle->config.DFF << SPI_CR1_DFF);
	tempReg |= (pSPIHandle->config.CPOL << SPI_CR1_CPOL);
	tempReg |= (pSPIHandle->config.CPHA << SPI_CR1_CPHA);
	tempReg |= (pSPIHandle->config.SSM << SPI_CR1_SSM);

	pSPIHandle->pSPIx->CR1 = tempReg;
}

void SPI_DeInit(SPI_RegDef_t *pSPIx) {
	if (pSPIx == SPI1) {
		SPI1_REG_RESET();
	} else if (pSPIx == SPI2) {
		SPI2_REG_RESET();
	} else if (pSPIx == SPI3) {
		SPI3_REG_RESET();
	}

	// TODO: Implement SPI disable, see reference manual for disabling the SPI
}

/*
 * Data send and receive
 */

uint8_t SPI_GetStatusFlag(SPI_RegDef_t *pSPIx, uint32_t mask) {
	if (pSPIx->SR & mask) {
		return FLAG_SET;
	}
	return FLAG_RESET;
}

// Blocking/polling call
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len) {
	while (len > 0) {
		// wait for TXE to be 1
		while (SPI_GetStatusFlag(pSPIx, SPI_FLAG_TXE) == FLAG_RESET)
			;
		if (pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
			// 16-bit DFF
			// typecast to uint16 to get 2 bytes of data every increment
			pSPIx->DR = *((uint16_t*) pTxBuffer);
			len--; // 2 bytes sent
			len--;
			(uint16_t*) pTxBuffer++;
		} else {
			// 8-bit DFF
			pSPIx->DR = *pTxBuffer;
			len--; // 1 byte sent
			pTxBuffer++;
		}
	}
}

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len) {
	while (len > 0) {
		// wait for TXE to be 1
		while (SPI_GetStatusFlag(pSPIx, SPI_FLAG_RXNE) == FLAG_RESET)
			;
		if (pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
			// 16-bit DFF
			// typecast to uint16 to get 2 bytes of data every increment
			*((uint16_t*) pRxBuffer) = pSPIx->DR;
			len--; // 2 bytes sent
			len--;
			(uint16_t*) pRxBuffer++;
		} else {
			// 8-bit DFF
			*pRxBuffer = pSPIx->DR;
			len--; // 1 byte sent
			pRxBuffer++;
		}
	}
}

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t enable) {
	if (enable == ENABLE) {
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	} else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

// make NSS signal internal high and avoids MODF error
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t enable) {
	if (enable == ENABLE) {
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	} else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t enable) {
	if (enable == ENABLE) {
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	} else {
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}

/*
 * IRQ config and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t irqNumber, uint8_t enable) {
	// copied from GPIO driver

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

void SPI_IRQPriorityConfig(uint8_t irqNumber, uint8_t irqPriority) {
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

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer,
		uint32_t size) {
	uint8_t state = pSPIHandle->txState;

	if (state == SPI_BUSY_IN_TX)
		return state;

	// 1. save tx buffer addr and len in SPI handle
	pSPIHandle->pTxBuffer = pTxBuffer;
	pSPIHandle->txLen = size;

	// 2. mark SPI state as busy in tx
	pSPIHandle->txState = SPI_BUSY_IN_TX;

	// 3. enable TXEIE control bit to get interrupt
	pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);

	return state;
}

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer,
		uint32_t size) {
	uint8_t state = pSPIHandle->rxState;

	if (state == SPI_BUSY_IN_RX)
		return state;

	// 1. save rx buffer addr and len in SPI handle
	pSPIHandle->pRxBuffer = pRxBuffer;
	pSPIHandle->rxLen = size;

	// 2. mark SPI state as busy in rx
	pSPIHandle->rxState = SPI_BUSY_IN_RX;

	// 3. enable RXEIE control bit to get interrupt
	pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

	return state;
}

void SPI_IRQHandling(SPI_Handle_t *pSPIHandle) {
	uint8_t statusFlag, interruptFlag;

	// check for txne
	statusFlag = pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	interruptFlag = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	// handle txe
	if (statusFlag && interruptFlag) {
		SPI_TXE_IT_Handle(pSPIHandle);
	}

	// check for rxne
	statusFlag = pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	interruptFlag = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

	// handle rxne
	if (statusFlag && interruptFlag) {
		SPI_RXNE_IT_Handle(pSPIHandle);
	}

	statusFlag = pSPIHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	interruptFlag = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

	// handle rxne
	if (statusFlag && interruptFlag) {
		SPI_OVR_ERR_IT_Handle(pSPIHandle);
	}
}

void SPI_TXE_IT_Handle(SPI_Handle_t *pSPIHandle) {
	if (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
		// 16-bit DFF
		// typecast to uint16 to get 2 bytes of data every increment
		pSPIHandle->pSPIx->DR = *((uint16_t*) pSPIHandle->pTxBuffer);
		pSPIHandle->txLen--; // 2 bytes sent
		pSPIHandle->txLen--; // 2 bytes sent
		(uint16_t*) pSPIHandle->pTxBuffer++;
	} else {
		// 8-bit DFF
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->txLen--;	// 1 byte sent
		pSPIHandle->pTxBuffer++;
	}

	if (!pSPIHandle->txLen) {
		// TX is complete, close SPI tx and call app callback
		SPI_CloseTx(pSPIHandle);
		SPI_AppEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}

void SPI_RXNE_IT_Handle(SPI_Handle_t *pSPIHandle) {
	if (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
		// 16-bit DFF
		// typecast to uint16 to get 2 bytes of data every increment
		*((uint16_t*) pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
		pSPIHandle->rxLen -= 2; // 2 bytes sent
		(uint16_t*) pSPIHandle->pRxBuffer++;
	} else {
		// 8-bit DFF
		*pSPIHandle->pRxBuffer = pSPIHandle->pSPIx->DR;
		pSPIHandle->rxLen--; // 1 byte sent
		pSPIHandle->pRxBuffer++;
	}

	if (!pSPIHandle->rxLen) {
		// RX is complete, call app callback
		SPI_CloseRx(pSPIHandle);
		SPI_AppEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}
void SPI_OVR_ERR_IT_Handle(SPI_Handle_t *pSPIHandle) {
	// clear ovr flag
	if (pSPIHandle->txState != SPI_BUSY_IN_TX) {
		SPI_ClearOVRFlag(pSPIHandle->pSPIx);
	}

	// call app callback
	SPI_AppEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx) {
	// read DR and SR to clear the OVR flag
	uint8_t temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void) temp; // suppress compiler warning
}

void SPI_CloseTx(SPI_Handle_t *pSPIHandle) {
	// clear txei register
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);

	// clear the global fields
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->txLen = 0;
	pSPIHandle->txState = SPI_READY;

}

void SPI_CloseRx(SPI_Handle_t *pSPIHandle) {
	// clear RXNEIE register
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);

	// clear the global fields
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->rxLen = 0;
	pSPIHandle->rxState = SPI_READY;
}

_weak void SPI_AppEventCallback(SPI_Handle_t *pSPIHandle, uint8_t event) {
	// weak implementation
}
