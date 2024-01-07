/*
 * utils.c
 *
 *  Created on: Jan 7, 2024
 *      Author: sf
 */

#include "utils.h"

#include <stdint.h>

void delay(void) {
	for (uint32_t i = 0; i < 500000; i++)
		;
}
