/*
 * example2_led_button_ext.c
 *
 *  Created on: Jan 10, 2024
 *      Author: sf
 */

#include "example2_led_button_ext.h"

#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"

#include "../utils.h"

#define BTN_PRESSED 0

void led_button_ext(void) {
	GPIO_Handle_t gpio_led, gpio_btn;

	gpio_led.pGPIOx = GPIOA;
	gpio_led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	gpio_led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpio_led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpio_led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpio_led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NO;

	GPIO_PeriClockCtrl(GPIOA, ENABLE);
	GPIO_Init(&gpio_led);

	gpio_btn.pGPIOx = GPIOB;
	gpio_btn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	gpio_btn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	gpio_btn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpio_btn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_PU;

	GPIO_PeriClockCtrl(GPIOB, ENABLE);
	GPIO_Init(&gpio_btn);

	for (;;) {
		if (GPIO_ReadInputPin(GPIOB, GPIO_PIN_NO_12) == BTN_PRESSED) {
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_7);
			delay(2);
		}

	}

}
