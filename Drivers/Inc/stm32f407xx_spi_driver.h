/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: Jan 18, 2024
 *      Author: sf
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"

typedef struct SPI_Config_t {
	uint8_t DeviceMode; // full/half-duplex/simplex mode
	uint8_t BusConfig;
	uint8_t DFF;
	uint8_t CPHA;
	uint8_t CPOL;
	uint8_t SSM;
	uint8_t Speed;
} SPI_Config_t;

typedef struct SPI_Handle_t {
	SPI_RegDef_t *pSPIx;
	SPI_Config_t config;
} SPI_Handle_t;

#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
