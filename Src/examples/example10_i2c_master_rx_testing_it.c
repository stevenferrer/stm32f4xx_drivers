/*
 * example8_i2c_master_tx_testing.c
 *
 *  Created on: Feb 9, 2024
 *      Author: sf
 */

#include <stdio.h>
#include <string.h>

#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_i2c_driver.h"

#include "../utils.h"

uint8_t rxComplt = RESET;

#define MY_ADDR 0x61
#define SLAVE_ADDR 0x68

I2C_Handle_t i2c1Handle;

// receive buffer
uint8_t receiveBuffer[32];

/*
 * PB6 - scl
 * PB9 - sda
 */

void I2C1_GPIO_Init(void) {
	GPIO_Handle_t i2cPins;

	i2cPins.pGPIOx = GPIOB;
	i2cPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	i2cPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	i2cPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	i2cPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_PU;
	i2cPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// scl
	i2cPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&i2cPins);

	// sda
	i2cPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
	GPIO_Init(&i2cPins);
}

void I2C1_Init(void) {
	i2c1Handle.i2cx = I2C1;
	i2c1Handle.config.ACKControl = I2C_ACK_ENABLE;
	i2c1Handle.config.DeviceAddress = MY_ADDR;
	i2c1Handle.config.FMDutyCycle = I2C_FM_DUTY_2;
	i2c1Handle.config.SCLSpeed = I2C_SCL_SPEED_STD;

	I2C_Init(&i2c1Handle);
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

int i2c_master_rx_it(void) {
	uint8_t commandCode;
	uint8_t len;

	I2C1_GPIO_Init();

	GPIO_Button_Init();

	I2C1_Init();

	// i2c IRQ configs
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV, ENABLE);
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER, ENABLE);

	// enable i2c peripheral
	I2C_PeripheralControl(I2C1, ENABLE);

	// CR1_ACK bit is only set after CR1_PE=1
	I2C_EnableAcking(I2C1, I2C_ACK_ENABLE);

	while (1) {
		if (GPIO_ReadInputPin(GPIOD, GPIO_PIN_NO_5) == 0) {
			delay(2);

			commandCode = 0x51;
			while (I2C_MasterSendDataIT(&i2c1Handle, &commandCode, 1,
			SLAVE_ADDR,
			I2C_SR_DISABLE) != I2C_STATE_READY) {
			}

//			while (I2C_MasterReceiveDataIT(&i2c1Handle, &len, 1, SLAVE_ADDR,
//			I2C_SR_DISABLE) != I2C_STATE_READY) {
//			}

//			commandCode = 0x52;
//			while (I2C_MasterSendDataIT(&i2c1Handle, &commandCode, 1,
//			SLAVE_ADDR,
//			I2C_SR_ENABLE) != I2C_STATE_READY)
//				;
//
//			while (I2C_MasterReceiveDataIT(&i2c1Handle, receiveBuffer, len,
//			SLAVE_ADDR,
//			I2C_SR_DISABLE) != I2C_STATE_READY)
//				;
//
//			rxComplt = RESET;
//
//			//wait till rx completes
//			while (rxComplt != SET) {
//
//			}
//
//			receiveBuffer[len + 1] = '\0';
//
//			// Note: Remove when not in debug mode
//			printf("Received from arduino: %s\n", receiveBuffer);
//			rxComplt = RESET;
		}
	}
}

void I2C1_EV_IRQHandler(void) {
	I2C_EV_IRQHandling(&i2c1Handle);
}

void I2C1_ER_IRQHandler(void) {
	I2C_ER_IRQHandling(&i2c1Handle);
}

void I2C_AppEventCallback(I2C_Handle_t *pI2CHandle, uint8_t event) {
	if (event == I2C_EV_TX_CMPLT) {
		printf("Tx is completed\n");
	} else if (event == I2C_EV_RX_CMPLT) {
		printf("Rx is completed\n");
		rxComplt = SET;
	} else if (event == I2C_ERROR_AF) {
		printf("Error : Ack failure\n");
		//in master ack failure happens when slave fails to send ack for the byte
		//sent from the master.
		I2C_CloseSendData(pI2CHandle);

		//generate the stop condition to release the bus
		I2C_GenerateStopCondition(I2C1);

		//Hang in infinite loop to prevent executing other codes
		// Note: This could be better!!!
		while (1)
			;
	}
}
