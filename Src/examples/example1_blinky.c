#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"

#include "../utils.h"

void blink(void) {
	GPIO_Handle_t gpio_led;

	gpio_led.pGPIOx = GPIOA;
	gpio_led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_1;
	gpio_led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpio_led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpio_led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpio_led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NO;

	GPIO_PeriClockCtrl(GPIOA, ENABLE);

	GPIO_Init(&gpio_led);

	for (;;) {
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_1);
		delay(2);
	}

}
