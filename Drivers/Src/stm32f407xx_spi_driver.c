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
}
void SPI_DeInit(SPI_RegDef_t *pSPIx) {
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
