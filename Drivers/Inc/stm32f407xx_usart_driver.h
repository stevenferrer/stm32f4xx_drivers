/*
 * stm32f407xx_usart_driver.h
 *
 *  Created on: Feb 29, 2024
 *      Author: sf
 */

#ifndef INC_STM32F407XX_USART_DRIVER_H_
#define INC_STM32F407XX_USART_DRIVER_H_

#include "stm32f407xx.h"

typedef struct USART_Config_t {
	uint8_t Mode;
	uint8_t Baud;
	uint8_t NoOfStopBits;
	uint8_t WordLen;
	uint8_t ParityControl;
	uint8_t HWFlowControl;
};

typedef struct USART_Handle_t {
	USART_RegDef_t *usartx;
	USART_Config_t config;
};

#endif /* INC_STM32F407XX_USART_DRIVER_H_ */
