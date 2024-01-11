/*
 * example2_blinky_ext.c
 *
 *  Created on: Jan 11, 2024
 *      Author: sf
 */

#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"

#include "../utils.h"

#include "example1_blinky_ext.h"

void blink_ext(void) {
	GPIO_Handle_t gpio_led;

	gpio_led.pGPIOx = GPIOB;
	gpio_led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	gpio_led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpio_led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpio_led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpio_led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_PU;

	GPIO_PeriClockCtrl(GPIOB, ENABLE);

	GPIO_Init(&gpio_led);

	for (;;) {
		GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_NO_12);
		delay(2);
	}

}
