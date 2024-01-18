/*
 * example3_button_interrupt.c
 *
 *  Created on: Jan 17, 2024
 *      Author: sf
 */

#include <string.h>

#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"

#include "example3_button_interrupt.h"

#include "../utils.h"

void button_interrupt(void) {
	GPIO_Handle_t gpio_led1, gpio_led2, gpio_btn;
	// initialize every member to zero
	memset(&gpio_led1, 0, sizeof(gpio_led1));
	memset(&gpio_btn, 0, sizeof(gpio_btn));

	GPIO_PeriClockCtrl(GPIOA, ENABLE);
	GPIO_PeriClockCtrl(GPIOD, ENABLE);

	gpio_led1.pGPIOx = GPIOD;
	gpio_led1.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	gpio_led1.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpio_led1.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpio_led1.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpio_led1.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NO;
	GPIO_Init(&gpio_led1);

	gpio_led2.pGPIOx = GPIOA;
	gpio_led2.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_1;
	gpio_led2.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpio_led2.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpio_led2.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpio_led2.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NO;
	GPIO_Init(&gpio_led2);
	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_1);

	gpio_btn.pGPIOx = GPIOD;
	gpio_btn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	gpio_btn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	gpio_btn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpio_btn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_PU;
	GPIO_Init(&gpio_btn);

	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRIORITY_15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);

	for (;;) {
	}

}

// Make sure to include GPIO_IRQHandling
void EXTI9_5_IRQHandler(void) {
	GPIO_IRQHandling(GPIO_PIN_NO_5);
	delay(3);
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_1);
}

