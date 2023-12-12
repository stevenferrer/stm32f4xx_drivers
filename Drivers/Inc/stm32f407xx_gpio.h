/*
 * stm32f407xx_gpio.h
 *
 */

#ifndef INC_STM32F407XX_GPIO_H_
#define INC_STM32F407XX_GPIO_H_

#include "stm32f407xx.h"

typedef struct GPIO_PinConfig_t {
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
} GPIO_PinConfig_t;

typedef struct GPIO_Handle_t {
	GPIO_RegDef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;
} GPIO_Handle_t;

void GPIO_Init(void);
void GPIO_DeInit(void);
void GPIO_PClockCtrl(void);

void GPIO_ReadInputPin(void);
void GPIO_ReadInputPort(void);
void GPIO_WriteOutputPin(void);
void GPIO_WriteOutputPort(void);
void GPIO_ToggleOutputPin(void);

void GPIO_IRQConfig(void);
void GPIO_IRQHandling(void);

#endif /* INC_STM32F407XX_GPIO_H_ */
