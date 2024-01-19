/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Jan 18, 2024
 *      Author: sf
 */

#include "stm32f407xx_spi_driver.h"

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
		SPI2_RESET_RESET();
	} else if (pSPIx == SPI3) {
		SPI3_REG_RESET();
	}
}

/*
 * Data send and receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t size) {
}
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t size) {
}

/*
 * IRQ config and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t irqNumber, uint8_t enable) {
}
void SPI_IRQPriorityConfig(uint8_t irqNumber, uint8_t irqPriority) {
}
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle) {
}
