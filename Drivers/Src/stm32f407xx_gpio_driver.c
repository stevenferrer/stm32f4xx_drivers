/*
 * stm32f407xx_gpio.c
 *
 */

#include <stm32f407xx_gpio_driver.h>

void GPIO_PeriClockCtrl(GPIO_RegDef_t *pGPIOx, uint8_t enable) {
	if (enable == ENABLE) {
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_EN();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_EN();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_EN();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_EN();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_EN();
		} else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_EN();
		} else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_EN();
		} else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_EN();
		} else if (pGPIOx == GPIOI) {
			GPIOI_PCLK_EN();
		}
	} else {
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_DI();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_DI();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_DI();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_DI();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_DI();
		} else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_DI();
		} else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_DI();
		} else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_DI();
		} else if (pGPIOx == GPIOI) {
			GPIOI_PCLK_DI();
		}
	}

}

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
	uint32_t temp = 0;
	const uint8_t pinNumber = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;

	// 1. configure the mode of GPIO pin
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
		// non-interrupt mode
		// multiply by 2 bc each pin uses 2 bit fields
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pinNumber); // clear register
		pGPIOHandle->pGPIOx->MODER |= temp; // set
	} else {
		// interrupt mode

		// 1. configure FTSR/RTSR or both
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT) {
			// configure FTSR
			EXTI->FTSR |= (1 << pinNumber);
			// clear corresponding RTSR bit (why tho?)
			EXTI->RTSR &= ~(1 << pinNumber);
		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT) {
			// configure RTSR
			EXTI->RTSR |= (1 << pinNumber);
			// clear corresponding FTSR bit (why tho?)
			EXTI->FTSR &= ~(1 << pinNumber);
		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT) {
			// configure both FTSR and RTSR
			EXTI->FTSR |= (1 << pinNumber);
			EXTI->RTSR |= (1 << pinNumber);
		}

		// 2. configure GPIO port selection in SYSCFG_EXTICR
		uint8_t extiCrPos = pinNumber / 4;
		uint8_t bitFieldOffset = (pinNumber % 4) * 4;

		// use macro to convert gpio base addr to port code
		uint8_t portCode = GPIO_BASE_ADDR_TO_PORT_CODE(pGPIOHandle->pGPIOx);

		SYSCFG_PCLK_EN(); // enable clock
		SYSCFG->EXTICR[extiCrPos] = portCode << (bitFieldOffset * 4);

		// 3. enable exti interrupt delivery using IMR
		EXTI->IMR |= (1 << pinNumber);
	}

	// 2. configure speed
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pinNumber); // clear register
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	// 3. configure pull-up/pull-down register
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pinNumber);
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pinNumber); // clear register
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	// 4. configure op-type
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pinNumber;
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pinNumber); // clear register
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	// 5. configure alternate functionality
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN) {
		uint8_t afrPos, afrPin;

		afrPos = pinNumber / 8;
		afrPin = pinNumber % 8;

		pGPIOHandle->pGPIOx->AFR[afrPos] &= ~(0xf << afrPin); // clear register
		pGPIOHandle->pGPIOx->AFR[afrPos] |=
				pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * afrPin);
	}
}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {
	if (pGPIOx == GPIOA) {
		GPIOA_REG_RESET();
	} else if (pGPIOx == GPIOB) {
		GPIOB_REG_RESET();
	} else if (pGPIOx == GPIOC) {
		GPIOC_REG_RESET();
	} else if (pGPIOx == GPIOD) {
		GPIOD_REG_RESET();
	} else if (pGPIOx == GPIOE) {
		GPIOE_REG_RESET();
	} else if (pGPIOx == GPIOF) {
		GPIOF_REG_RESET();
	} else if (pGPIOx == GPIOG) {
		GPIOG_REG_RESET();
	} else if (pGPIOx == GPIOH) {
		GPIOH_REG_RESET();
	} else if (pGPIOx == GPIOI) {
		GPIOI_REG_RESET();
	}
}

uint8_t GPIO_ReadInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber) {
	/*
	 * - left-shift by pinNumber (e.g bit 8 will be shifted to first bit)
	 * - mask the register by 0x1
	 * - cast to uint8
	 * - returns 0 or 1
	 */
	return (uint8_t) ((pGPIOx->IDR >> pinNumber) & 0x00000001);
}

uint16_t GPIO_ReadInputPort(GPIO_RegDef_t *pGPIOx) {
	return (uint16_t) (pGPIOx->IDR);
}

void GPIO_WriteOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber,
		uint8_t value) {

	if (value == SET) {
		// write 1 to ODR at pin bit field
		pGPIOx->ODR |= (0x1 << pinNumber);

	} else {
		pGPIOx->ODR |= ~(0x1 << pinNumber);
	}
}

void GPIO_WriteOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value) {
	pGPIOx->ODR = value;
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber) {
	pGPIOx->ODR ^= (1 << pinNumber);
}

void GPIO_IRQConfig(uint8_t irqNumber, uint8_t irqPriority, uint8_t enable) {
}

void GPIO_IRQHandling(uint8_t pinNumber) {
}
