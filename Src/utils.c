/*
 * utils.c
 *
 *  Created on: Jan 7, 2024
 *      Author: sf
 */

#include "utils.h"

#include <stdint.h>

#define HIGH 1
#define LOW 0

#define DELAY_NUM 500000

void delay(uint32_t div) {
	for (uint32_t i = 0; i < DELAY_NUM/div; i++)
		;
}
