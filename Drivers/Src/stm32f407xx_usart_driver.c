/*
 * stm32f407xx_usart_driver.c
 *
 *  Created on: Feb 29, 2024
 *      Author: sf
 */

#include "stm32f407xx.h"
#include "stm32f407xx_usart_driver.h"
#include "stm32f407xx_rcc_driver.h"

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
	 * CR1 configuration
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
		tempreg |= (1 << USART_CR3_CTSE);
		tempreg |= (1 << USART_CR3_RTSE);
	}

	pUSARTHandle->usartx->CR3 |= tempreg;

	/*
	 * BRR configuration
	 */

	USART_SetBaudRate(pUSARTHandle->usartx, pUSARTHandle->config.Baud);
}

void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate) {

	//Variable to hold the APB clock
	uint32_t PCLKx;

	uint32_t usartdiv;

	//variables to hold Mantissa and Fraction values
	uint32_t M_part, F_part;

	uint32_t tempreg = 0;

	//Get the value of APB bus clock in to the variable PCLKx
	if (pUSARTx == USART1 || pUSARTx == USART6) {
		//USART1 and USART6 are hanging on APB2 bus
		PCLKx = RCC_GetPclk2Value();
	} else {
		PCLKx = RCC_GetPclk1Value();
	}

	//Check for OVER8 configuration bit
	if (pUSARTx->CR1 & (1 << USART_CR1_OVER8)) {
		// Note: The formula was originally multiplied by 100 to
		// get whole numbers instead of dealing with floating point.
		// The code below was simplified by dividing 4 to both
		// numerator and denominator

		//OVER8 = 1 , over sampling by 8
		usartdiv = ((25 * PCLKx) / (2 * BaudRate));
	} else {
		//over sampling by 16
		usartdiv = ((25 * PCLKx) / (4 * BaudRate));
	}

	//Calculate the Mantissa part
	M_part = usartdiv / 100;

	//Place the Mantissa part in appropriate bit position . refer USART_BRR
	tempreg |= M_part << USART_BRR_DIV_MANTISSA;

	//Extract the fraction part
	F_part = (usartdiv - (M_part * 100));

	//Calculate the final fractional
	if (pUSARTx->CR1 & (1 << USART_CR1_OVER8)) {
		//OVER8 = 1 , over sampling by 8
		// Note: The mask 3-bits (0x07)
		F_part = (((F_part * 8) + 50) / 100) & ((uint8_t) 0x07);

	} else {
		//over sampling by 16
		// mask 4-bits (0x0f)
		F_part = (((F_part * 16) + 50) / 100) & ((uint8_t) 0x0F);

	}

	//Place the fractional part in appropriate bit position . refer USART_BRR
	tempreg |= F_part; // Fractional part is at 0th position

	//copy the value of tempreg in to BRR register
	pUSARTx->BRR = tempreg;
}

void USART_DeInit(USART_RegDef_t *pUSARTx) {

}

/*
 * Data Send and Receive
 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer,
		uint32_t len) {
	uint16_t *pdata;
	// Loop over until len number of bytes are transferred
	for (uint32_t i = 0; i < len; i++) {
		// Wait until TXE flag is set in the SR
		while (!USART_GetFlagStatus(pUSARTHandle->usartx, USART_SR_TXE))
			;

		// Check the USART_WordLength item for 9BIT or 8BIT in a frame
		if (pUSARTHandle->config.WordLength == USART_WORDLEN_9BITS) {
			// if 9BIT, load the DR with 2bytes masking the bits other than first 9 bits
			// masking is done to make sure only 9-bits (0x01ff) is written
			pdata = (uint16_t*) pTxBuffer;
			pUSARTHandle->usartx->DR = (*pdata & (uint16_t) 0x01FF);

			// check for USART_ParityControl
			if (pUSARTHandle->config.ParityControl == USART_PARITY_DISABLE) {
				// No parity is used in this transfer. so, 9bits of user data will be sent
				// Increment pTxBuffer twice
				pTxBuffer++;
				pTxBuffer++;
			} else {
				// Parity bit is used in this transfer . so , 8bits of user data will be sent
				// The 9th bit will be replaced by parity bit by the hardware
				pTxBuffer++;
			}
		} else {
			// This is 8bit data transfer
			// Note: Cast to `uint8_t` seems redundant as per course comments.
			pUSARTHandle->usartx->DR = (*pTxBuffer & (uint8_t) 0xFF);

			// Increment the buffer address
			pTxBuffer++;
		}
	}

	//Implement the code to wait till TC flag is set in the SR
	while (!USART_GetFlagStatus(pUSARTHandle->usartx, USART_SR_TC))
		;
}

void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer,
		uint32_t len) {

	//Loop over until "len" number of bytes are transferred
	for (uint32_t i = 0; i < len; i++) {
		//Implement the code to wait until RXNE flag is set in the SR
		while (!USART_GetFlagStatus(pUSARTHandle->usartx, USART_SR_RXNE))
			;

		//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
		if (pUSARTHandle->config.WordLength == USART_WORDLEN_9BITS) {
			//We are going to receive 9bit data in a frame

			//check are we using USART_ParityControl control or not
			if (pUSARTHandle->config.ParityControl == USART_PARITY_DISABLE) {
				//No parity is used. so, all 9bits will be of user data

				//read only first 9 bits. so, mask the DR with 0x01FF
				*((uint16_t*) pRxBuffer) = (pUSARTHandle->usartx->DR
						& (uint16_t) 0x01FF);

				//Now increment the pRxBuffer two times
				pRxBuffer++;
				pRxBuffer++;
			} else {
				//Parity is used, so, 8bits will be of user data and 1 bit is parity
				*pRxBuffer = (pUSARTHandle->usartx->DR & (uint8_t) 0xFF);

				//Increment the pRxBuffer
				pRxBuffer++;
			}
		} else {
			//We are going to receive 8bit data in a frame

			//check are we using USART_ParityControl control or not
			if (pUSARTHandle->config.ParityControl == USART_PARITY_DISABLE) {
				//No parity is used , so all 8bits will be of user data

				//read 8 bits from DR
				*pRxBuffer = (uint8_t) (pUSARTHandle->usartx->DR
						& (uint8_t) 0xFF);
			}

			else {
				//Parity is used, so , 7 bits will be of user data and 1 bit is parity

				//read only 7 bits , hence mask the DR with 0X7F
				*pRxBuffer = (uint8_t) (pUSARTHandle->usartx->DR
						& (uint8_t) 0x7f);

			}

			//increment the pRxBuffer
			pRxBuffer++;
		}
	}
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
