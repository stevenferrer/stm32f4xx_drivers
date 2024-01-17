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
