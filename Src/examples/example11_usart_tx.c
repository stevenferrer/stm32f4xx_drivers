/*
 * example11_usart_tx.c
 *
 *  Created on: Mar 7, 2024
 *      Author: sf
 */

#include <stdio.h>
#include <string.h>

#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_usart_driver.h"

#include "../utils.h"

#define MY_ADDR 0x61
#define SLAVE_ADDR 0x68

USART_Handle_t usart1Handle;

// receive buffer
char msg[1024] = "Hello, USART!\n\r";

/*
 * PA9 - tx
 * PA10 - rx
 */

void USART1_GPIO_Init(void) {
	GPIO_Handle_t usartPins;

	usartPins.pGPIOx = GPIOA;
	usartPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	usartPins.GPIO_PinConfig.GPIO_PinAltFunMode = 7;
	usartPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	usartPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_PU;
	usartPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// tx
	usartPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
	GPIO_Init(&usartPins);

	// rx
	usartPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_10;
	GPIO_Init(&usartPins);
}

void USART2_Init(void) {
	usart1Handle.usartx = USART1;
	usart1Handle.config.Baud = USART_STD_BAUD_115200;
	usart1Handle.config.HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	usart1Handle.config.Mode = USART_MODE_ONLY_TX;
	usart1Handle.config.NoOfStopBits = USART_STOPBITS_1;
	usart1Handle.config.WordLength = USART_WORDLEN_8BITS;
	usart1Handle.config.ParityControl = USART_PARITY_DISABLE;

	USART_Init(&usart1Handle);
}

void GPIO_Button_Init(void) {
	GPIO_Handle_t gpioBtn;

	gpioBtn.pGPIOx = GPIOD;
	gpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	gpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	gpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_PU;

	GPIO_Init(&gpioBtn);
}

void usart_tx(void) {
	USART1_GPIO_Init();

	USART2_Init();

	GPIO_Button_Init();

	USART_PeripheralControl(USART1, ENABLE);

	while (1) {
		if (GPIO_ReadInputPin(GPIOD, GPIO_PIN_NO_5) == 0) {
			delay(2);

			USART_SendData(&usart1Handle, (uint8_t*) msg, strlen(msg));
		}
	}
}
