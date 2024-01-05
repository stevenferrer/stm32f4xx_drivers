/*
 * stm32f407xx_gpio.h
 *
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

typedef struct GPIO_PinConfig_t {
	uint8_t GPIO_PinNumber; // use GPIO_PIN_*
	uint8_t GPIO_PinMode; // use GPIO_MODE_*
	uint8_t GPIO_PinSpeed; // use GPIO_SPEED_*
	uint8_t GPIO_PinPuPdControl; // use GPIO_PUPD_*
	uint8_t GPIO_PinOPType; // use GPIO_OP_TYPE_*
	uint8_t GPIO_PinAltFunMode;
} GPIO_PinConfig_t;

typedef struct GPIO_Handle_t {
	GPIO_RegDef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;
} GPIO_Handle_t;

void GPIO_PeriClockCtrl(GPIO_RegDef_t *pGPIOx, uint8_t enable);

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

uint8_t GPIO_ReadInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);
uint16_t GPIO_ReadInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber,
		uint8_t value);
void GPIO_WriteOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);

void GPIO_IRQConfig(uint8_t irqNumber, uint8_t irqPriority, uint8_t enable);
void GPIO_IRQHandling(uint8_t pinNumber);

// pins
#define GPIO_PIN_NO_0 0
#define GPIO_PIN_NO_1 1
#define GPIO_PIN_NO_2 2
#define GPIO_PIN_NO_3 3
#define GPIO_PIN_NO_4 4
#define GPIO_PIN_NO_5 5
#define GPIO_PIN_NO_6 6
#define GPIO_PIN_NO_7 7
#define GPIO_PIN_NO_8 8
#define GPIO_PIN_NO_9 9
#define GPIO_PIN_NO_10 10
#define GPIO_PIN_NO_11 11
#define GPIO_PIN_NO_12 12
#define GPIO_PIN_NO_13 13
#define GPIO_PIN_NO_14 14
#define GPIO_PIN_NO_15 15

// non-interrupt modes
#define GPIO_MODE_IN 0
#define GPIO_MODE_OUT 1
#define GPIO_MODE_ALTFN 2
#define GPIO_MODE_ANALOG 3

// interrupt modes
#define GPIO_MODE_IT_FT 4
#define GPIO_MODE_IT_RT 5
#define GPIO_MODE_IT_RFT 6

// op-type
#define GPIO_OP_TYPE_PP 0
#define GPIO_OP_TYPE_OD 1

// speed
#define GPIO_SPEED_LOW 0
#define GPIO_SPEED_MEDIUM 1
#define GPIO_SPEED_FAST 2
#define GPIO_SPEED_HIGH 3

// pull-up/pull-down
#define GPIO_PUPD_NO 0
#define GPIO_PUPD_PU 1
#define GPIO_PUPD_PD 2

#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
