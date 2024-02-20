/*
 * main.c
 *
 *  Created on: Jan 7, 2024
 *      Author: sf
 */
#include <stdio.h>
#include "examples/examples.h"

// TODO: Find a way to remove initialise_monitor_handles
// when not in debug mode

// Note: Comment when in Run mode
extern void initialise_monitor_handles();

int main(void) {
	initialise_monitor_handles();

	printf("lorem ipsum dolor sit amet.\n");

	i2c_master_rx();
	return 0;
}
