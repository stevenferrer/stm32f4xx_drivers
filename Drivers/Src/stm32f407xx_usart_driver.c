/*
 * stm32f407xx_usart_driver.c
 *
 *  Created on: Feb 29, 2024
 *      Author: sf
 */

#include "stm32f407xx.h"
#include "stm32f407xx_usart_driver.h"

/*
 * Peripheral Clock setup
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t enable) {
	if (enable == ENABLE) {
		if (pUSARTx == USART1) {
			USART1_PCLK_EN();
		}
	} else {
		if (pUSARTx == USART1) {
			USART1_PCLK_DI();
		}
	}
}

/*
 * Init and De-init
 */
void USART_Init(USART_Handle_t *pUSARTHandle) {
	//Temporary variable
	uint32_t tempreg = 0;

	/*
	 * CR1 configruation
	 */

	// Enable the Clock for given USART peripheral
	USART_PeriClockControl(pUSARTHandle->usartx, ENABLE);

	//Enable USART Tx and Rx engines according to the USART_Mode configuration item
	if (pUSARTHandle->config.Mode == USART_MODE_ONLY_RX) {
		// Enable the Receiver bit field
		tempreg |= (1 << USART_CR1_RE);
	} else if (pUSARTHandle->config.Mode == USART_MODE_ONLY_TX) {
		// Enable the Transmitter bit field
		tempreg |= (1 << USART_CR1_TE);

	} else if (pUSARTHandle->config.Mode == USART_MODE_TXRX) {
		// Enable the both Transmitter and Receiver bit fields
		tempreg |= ((1 << USART_CR1_RE) | (1 << USART_CR1_TE));
	}

	// Configure the Word length configuration item
	tempreg |= pUSARTHandle->config.WordLength << USART_CR1_M;

	// Configuration of parity control bit fields
	if (pUSARTHandle->config.ParityControl == USART_PARITY_EN_EVEN) {
		// Enable the parity control
		tempreg |= (1 << USART_CR1_PCE);

		// Even parity is enabled by default

	} else if (pUSARTHandle->config.ParityControl == USART_PARITY_EN_ODD) {
		// Enable the parity control
		tempreg |= (1 << USART_CR1_PCE);

		// Enable ODD parity
		tempreg |= (1 << USART_CR1_PS);

	}

	// Program the CR1 register
	pUSARTHandle->usartx->CR1 |= tempreg;

	/*
	 * CR2 configuration
	 */

	tempreg = 0;

	// Configure the number of stop bits inserted during USART frame transmission
	tempreg |= pUSARTHandle->config.NoOfStopBits << USART_CR2_STOP;

	// Program the CR2 register
	pUSARTHandle->usartx->CR2 |= tempreg;

	/*
	 * CR3 configuration
	 */

	tempreg = 0;

	// Configuration of USART hardware flow control
	if (pUSARTHandle->config.HWFlowControl == USART_HW_FLOW_CTRL_CTS) {
		// Enable CTS flow control
		tempreg |= (1 << USART_CR3_CTSE);

	} else if (pUSARTHandle->config.HWFlowControl == USART_HW_FLOW_CTRL_RTS) {
		// Enable RTS flow control
		tempreg |= (1 << USART_CR3_RTSE);

	} else if (pUSARTHandle->config.HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS) {
		// Enable both CTS and RTS Flow control
		tempreg |= ((1 << USART_CR3_CTSE) | (1 << USART_CR3_RTSE));
	}

	pUSARTHandle->usartx->CR3 |= tempreg;

	/*
	 * BRR configuration
	 */

	// TODO: Implement the code to configure the baud rate
}

void USART_DeInit(USART_RegDef_t *pUSARTx) {

}

/*
 * Data Send and Receive
 */
void USART_SendData(USART_RegDef_t *pUSARTx, uint8_t *pTxBuffer, uint32_t len) {

}

void USART_ReceiveData(USART_RegDef_t *pUSARTx, uint8_t *pRxBuffer,
		uint32_t len) {

}

uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer,
		uint32_t len) {
	return 0;
}

uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer,
		uint32_t len) {
	return 0;
}

/*
 * IRQ Configuration and ISR handling
 */
void USART_IRQInterruptConfig(uint8_t irqNumber, uint8_t enable) {
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

void USART_IRQPriorityConfig(uint8_t irqNumber, uint32_t irqPriority) {
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

void USART_IRQHandling(USART_Handle_t *pUSARTHandle) {

}

/*
 * Other Peripheral Control APIs
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t enable) {
	if (enable == ENABLE) {
		pUSARTx->CR1 |= (1 << USART_CR1_UE);
	} else {
		pUSARTx->CR1 &= ~(1 << USART_CR1_UE);
	}
}

uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint32_t mask) {
	if (pUSARTx->SR & mask) {
		return FLAG_SET;
	}
	return FLAG_RESET;
}

void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t flag) {

}

/*
 * Application callback
 */
_weak void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,
		uint8_t appEvent) {

}
