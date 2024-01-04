/*
 * stm32f407xx_gpio.c
 *
 */

#include <stm32f407xx_gpio_driver.h>

/*
 * @fn
 *
 * @breif
 *
 * @param[in]
 * @param[in]
 * @param[in]
 *
 * @return
 *
 * @note
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {
}

void GPIO_PeriClockCtrl(GPIO_RegDef_t *pGPIOx, uint8_t enable) {
	if (pGPIOx == GPIOA) {
		enable == ENABLE ? GPIOA_PCLK_EN() : GPIOA_PCLK_DI();
	} else if (pGPIOx == GPIOB) {
		enable == ENABLE ? GPIOB_PCLK_EN() : GPIOB_PCLK_DI();
	} else if (pGPIOx == GPIOC) {
		enable == ENABLE ? GPIOC_PCLK_EN() : GPIOC_PCLK_DI();
	} else if (pGPIOx == GPIOD) {
		enable == ENABLE ? GPIOD_PCLK_EN() : GPIOD_PCLK_DI();
	} else if (pGPIOx == GPIOE) {
		enable == ENABLE ? GPIOE_PCLK_EN() : GPIOE_PCLK_DI();
	} else if (pGPIOx == GPIOF) {
		enable == ENABLE ? GPIOF_PCLK_EN() : GPIOF_PCLK_DI();
	} else if (pGPIOx == GPIOG) {
		enable == ENABLE ? GPIOG_PCLK_EN() : GPIOG_PCLK_DI();
	} else if (pGPIOx == GPIOH) {
		enable == ENABLE ? GPIOH_PCLK_EN() : GPIOH_PCLK_DI();
	} else if (pGPIOx == GPIOI) {
		enable == ENABLE ? GPIOI_PCLK_EN() : GPIOI_PCLK_DI();
	}
}

void GPIO_DeInit(GPIO_Handle_t *pGPIOHandle) {
}

uint8_t GPIO_ReadInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber) {
	return 0;
}

uint16_t GPIO_ReadInputPort(GPIO_RegDef_t *pGPIOx) {
	return 0;
}

void GPIO_WriteOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber,
		uint8_t value) {
}

void GPIO_WriteOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value) {
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber) {
}

void GPIO_IRQConfig(uint8_t irqNumber, uint8_t irqPriority, uint8_t enable) {
}

void GPIO_IRQHandling(uint8_t pinNumber) {
}
