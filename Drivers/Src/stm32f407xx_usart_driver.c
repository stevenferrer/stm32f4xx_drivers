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

}

uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer,
		uint32_t len) {

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

}

uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint32_t mask) {
	return 0;
}

void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t flag) {

}

/*
 * Application callback
 */
_weak void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,
		uint8_t appEvent) {

}
