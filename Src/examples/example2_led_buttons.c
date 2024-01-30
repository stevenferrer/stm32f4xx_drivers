/*
 * example2_led_button_ext.c
 *
 *  Created on: Jan 10, 2024
 *      Author: sf
 */

#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"

#include "../utils.h"

void led_external_button(void) {
	GPIO_Handle_t gpio_led, gpio_btn;

//	gpio_led.pGPIOx = GPIOA;
//	gpio_led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
//	gpio_led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
//	gpio_led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
//	gpio_led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
//	gpio_led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NO;

//	GPIO_Init(&gpio_led);

	gpio_btn.pGPIOx = GPIOD;
	gpio_btn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	gpio_btn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	gpio_btn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpio_btn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_PU;

	GPIO_Init(&gpio_btn);

	for (;;) {
		uint8_t value = GPIO_ReadInputPin(GPIOD, GPIO_PIN_NO_5);
		if (value == 0) {
			delay(2);
		}

	}
}

void led_user_button(void) {
	GPIO_Handle_t gpio_led, gpio_btn;

	gpio_led.pGPIOx = GPIOA;
	gpio_led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_1;
	gpio_led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpio_led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpio_led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	gpio_led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NO;

	GPIO_Init(&gpio_led);

	gpio_btn.pGPIOx = GPIOA;
	gpio_btn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	gpio_btn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	gpio_btn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpio_btn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NO;

	GPIO_Init(&gpio_btn);

	while (1) {
		if (GPIO_ReadInputPin(GPIOA, GPIO_PIN_NO_0) == 0) {
			delay(2);
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_1);
		}

	}
}
