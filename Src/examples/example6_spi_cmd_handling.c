/*
 * example6_spi_cmd_handling.c
 *
 *  Created on: Jan 31, 2024
 *      Author: sf
 */

#include <string.h>

#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"

#include "../utils.h"

//command codes
#define COMMAND_LED_CTRL      		0x50
#define COMMAND_SENSOR_READ      	0x51
#define COMMAND_LED_READ      		0x52
#define COMMAND_PRINT      			0x53
#define COMMAND_ID_READ      		0x54

#define LED_ON     1
#define LED_OFF    0

//arduino analog pins
#define ANALOG_PIN0 	0
#define ANALOG_PIN1 	1
#define ANALOG_PIN2 	2
#define ANALOG_PIN3 	3
#define ANALOG_PIN4 	4

//arduino led
#define LED_PIN  9

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
	spiPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_PU;
	spiPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// sclk
	spiPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&spiPin);

	// mosi
	spiPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&spiPin);

	// miso
	spiPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&spiPin);
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

uint8_t SPI_VerifyAck(uint8_t ack) {
	return ack == 0xf5 ? 1 : 0;
}

void spi_cmd_handling(void) {
	uint8_t dummyWrite = 0xff;
	uint8_t dummyRead;
	GPIO_Button_Init();
	SPI2_GPIO_Init();
	SPI2_Init();

	SPI_SSOEConfig(SPI2, ENABLE);

	uint8_t led_status = 0;

	while (1) {
		uint8_t value = GPIO_ReadInputPin(GPIOD, GPIO_PIN_NO_5);
		if (value != 0)
			continue;

		delay(2);

		// enable SPI peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		// start: command send
		uint8_t commandCode = COMMAND_LED_CTRL;
		SPI_SendData(SPI2, &commandCode, 1);

		// Do dummy read to clear RXNE
		SPI_ReceiveData(SPI2, &dummyRead, 1);

		// Do dummy send to move slave response register
		uint8_t ackByte;
		uint8_t args[0];
		SPI_SendData(SPI2, &dummyWrite, 1);

		SPI_ReceiveData(SPI2, &ackByte, 1);

		if (SPI_VerifyAck(ackByte)) {
			led_status = !led_status;
			// send args
			args[0] = LED_PIN;
			args[1] = led_status;
			SPI_SendData(SPI2, args, 2);
		} else {

		}

		// end: command send

		//lets confirm SPI is not busy
		while (SPI_GetStatusFlag(SPI2, SPI_BUSY_FLAG))
			;

		//Disable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, DISABLE);
	}

}
