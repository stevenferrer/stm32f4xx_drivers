/*
 * main.c
 *
 *  Created on: Jan 7, 2024
 *      Author: sf
 */

#include "examples/example3_button_interrupt.h"

#include "stm32f407xx_gpio_driver.h"

#include "utils.h"

int main(void) {
	button_interrupt();
	return 0;
}

// Make sure to include GPIO_IRQHandling
void EXTI9_5_IRQHandler(void) {

	GPIO_IRQHandling(GPIO_PIN_NO_5);
	delay(3);
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
}
