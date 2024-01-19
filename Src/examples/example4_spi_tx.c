/*
 * example4_spi_tx.c
 *
 *  Created on: Jan 19, 2024
 *      Author: sf
 */

#include "example4_spi_tx.h"

#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"

#include <string.h>

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
//	// nss
//	spiPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
//	GPIO_Init(&spiPin);
}

void SPI2_Init(void) {
	SPI_Handle_t spi2Handle;

	spi2Handle.pSPIx = SPI2;
	spi2Handle.config.BusConfig = SPI_BUS_CONFIG_FD;
	spi2Handle.config.DeviceMode = SPI_MODE_DEVICE_MASTER;
	spi2Handle.config.SCLKSpeed = SPI_SCLK_SPEED_DIV_2;
	spi2Handle.config.DFF = SPI_DFF_8;
	spi2Handle.config.CPOL = SPI_CPOL_LOW;
	spi2Handle.config.CPHA = SPI_CPHA_LOW;
	spi2Handle.config.SSM = SPI_SSM_EN; // software-slave management enabled

	SPI_Init(&spi2Handle);
}

void spi_send_data(void) {
	char userData[] = "Hello, world!";
	SPI2_GPIO_Init();
	SPI2_Init();

	SPI_SSIConfig(SPI2, ENABLE);
	// enable SPI peripheral
	SPI_PeripheralControl(SPI2, ENABLE);

	SPI_SendData(SPI2, (uint8_t*) userData, strlen(userData));

	while (1) {
	}
}
