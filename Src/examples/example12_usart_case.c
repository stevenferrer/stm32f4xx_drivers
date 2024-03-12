/*
 * example12_usart_case.c
 *
 *  Created on: Mar 12, 2024
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
char msg[1024] = "zako";

//reply from arduino will be stored here
char rx_buf[1024];

//This flag indicates reception completion
uint8_t rxDone = RESET;
uint8_t txDone = RESET;

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

void USART1_Init(void) {
	usart1Handle.usartx = USART1;
	usart1Handle.config.Baud = USART_STD_BAUD_115200;
	usart1Handle.config.HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	usart1Handle.config.Mode = USART_MODE_TXRX;
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

void usart_case(void) {
	USART1_GPIO_Init();

	USART1_Init();

	GPIO_Button_Init();

	USART_IRQInterruptConfig(IRQ_NO_USART1, ENABLE);

	USART_PeripheralControl(USART1, ENABLE);

	printf("Application is running\n");

	while (1) {
		if (GPIO_ReadInputPin(GPIOD, GPIO_PIN_NO_5) == 0) {
			delay(2);

			//Send the msg
//			while (USART_SendDataIT(&usart1Handle, (uint8_t*) msg, strlen(msg))
//					!= USART_READY)
//				;
//
//			while (txDone != SET)
//				;
			USART_SendDataIT(&usart1Handle, (uint8_t*) msg, strlen(msg));

			printf("Transmitted: %s\n", msg);

			while (USART_ReceiveDataIT(&usart1Handle, (uint8_t*) rx_buf,
					strlen(msg)) != USART_READY)
				;

			while (rxDone != SET)
				;

			//just make sure that last byte should be null otherwise %s fails while printing
			rx_buf[strlen(msg) + 1] = '\0';

			//Print what we received from the arduino
			printf("Received: %s\n", rx_buf);

			//invalidate the flag
			rxDone = RESET;
		}
	}
}

void USART1_IRQHandler(void) {
	USART_IRQHandling(&usart1Handle);
}

void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t event) {
	if (event == USART_EVENT_RX_CMPLT) {
		rxDone = SET;
//		printf("rx done.\n");
	} else if (event == USART_EVENT_TX_CMPLT) {
		txDone = SET;
//		printf("tx done.\n");
	}
}
