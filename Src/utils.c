/*
 * utils.c
 *
 *  Created on: Jan 7, 2024
 *      Author: sf
 */

#include "utils.h"

#include <stdint.h>

#define DELAY_NUM 500000

void delay(void) {
	for (uint32_t i = 0; i < DELAY_NUM/2; i++)
		;
}
