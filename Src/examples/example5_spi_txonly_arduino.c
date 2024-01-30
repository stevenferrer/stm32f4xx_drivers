/*
 * example5_spi_txonly_arduino.c
 *
 *  Created on: Jan 29, 2024
 *      Author: sf
 */

#include <string.h>

#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"

#include "../utils.h"

/* SPI2 pins:
 * PB12 - nss
 * PB13 - sclk
 * PB14 - miso
 * PB15 - mosi
 * Alt function: 5
 */

void SPI2_GPIO_Init(void) {
	GPIO_Handle_t spiPin;

	spiPin.pGPIOx = GPIOB;
	spiPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	spiPin.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_AFR_5;
	spiPin.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	spiPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NO;
	spiPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// sclk
	spiPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&spiPin);

	// mosi
	spiPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&spiPin);

//	// miso
//	spiPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
//	GPIO_Init(&spiPin);
//
	// nss
	spiPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&spiPin);
}

void SPI2_Init(void) {
	SPI_Handle_t spi2Handle;

	spi2Handle.pSPIx = SPI2;
	spi2Handle.config.BusConfig = SPI_BUS_CONFIG_FD;
	spi2Handle.config.DeviceMode = SPI_MODE_DEVICE_MASTER;
	spi2Handle.config.SCLKSpeed = SPI_SCLK_SPEED_DIV_8;
	spi2Handle.config.DFF = SPI_DFF_8;
	spi2Handle.config.CPOL = SPI_CPOL_LOW;
	spi2Handle.config.CPHA = SPI_CPHA_LOW;
	spi2Handle.config.SSM = SPI_SSM_DI; // hardware-slave management enabled

	SPI_Init(&spi2Handle);
}

void GPIO_Button_Init(void) {
	GPIO_Handle_t gpio_btn;

	gpio_btn.pGPIOx = GPIOD;
	gpio_btn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	gpio_btn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	gpio_btn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpio_btn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_PU;

	GPIO_Init(&gpio_btn);
}

void spi_send_data_arduino(void) {
	char userData[] = "Lorem ipsum dolor sit amet, consectetur adipiscing elit. Nam lacinia odio dui, et pellentesque neque rutrum non. Sed et nulla in felis malesuada auctor a nec diam. Quisque in interdum ligula. Vestibulum vehicula lorem felis, quis scelerisque augue.";

	GPIO_Button_Init();
	SPI2_GPIO_Init();
	SPI2_Init();

	SPI_SSOEConfig(SPI2, ENABLE);

	while (1) {
		uint8_t value = GPIO_ReadInputPin(GPIOD, GPIO_PIN_NO_5);
		if (value != 0)
			continue;

		delay(2);

		// enable SPI peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		// send length info to slave device
		uint8_t data_len = strlen(userData);
		SPI_SendData(SPI2, &data_len, 1);

		SPI_SendData(SPI2, (uint8_t*) userData, strlen(userData));

		//lets confirm SPI is not busy
		while (SPI_GetStatusFlag(SPI2, SPI_BUSY_FLAG))
			;

		//Disable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, DISABLE);
	}

}
