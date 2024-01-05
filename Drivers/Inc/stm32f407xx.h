/*
 * stm32f407xx.h
 *
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

/*
 * Memory base addresses
 */
#define FLASH_BASE_ADDR 0x08000000u // main memory base address
#define SRAM1_BASE_ADDR 0x20000000u // sram1 base address
#define SRAM2_BASE_ADDR 0x2001C000u // sram2 base address
#define ROM_BASE_ADDR 0x1fff0000u // system memory base address
#define SRAM SRAM1_BASE_ADDR

/*
 * AHBx and APBx base addresses
 */
#define PERI_BASE_ADDR 0x40000000u
#define APB1_BASE_ADDR PERI_BASE_ADDR
#define APB2_BASE_ADDR 0x40010000u
#define AHB1_BASE_ADDR 0x40020000u
#define AHB2_BASE_ADDR 0x50000000u

/*
 * AHB1 peripheral base addresses
 */

#define GPIOA_BASE_ADDR (AHB1_BASE_ADDR + 0x0u)
#define GPIOB_BASE_ADDR (AHB1_BASE_ADDR + 0x04u)
#define GPIOC_BASE_ADDR (AHB1_BASE_ADDR + 0x08u)
#define GPIOD_BASE_ADDR (AHB1_BASE_ADDR + 0x0cu)
#define GPIOE_BASE_ADDR (AHB1_BASE_ADDR + 0x10u)
#define GPIOF_BASE_ADDR (AHB1_BASE_ADDR + 0x14u)
#define GPIOG_BASE_ADDR (AHB1_BASE_ADDR + 0x18u)
#define GPIOH_BASE_ADDR (AHB1_BASE_ADDR + 0x1cu)
#define GPIOI_BASE_ADDR (AHB1_BASE_ADDR + 0x20u)

#define RCC_BASE_ADDR (AHB1_BASE_ADDR + 0x38u)

/*
 * APB1 peripheral base addresses
 */

#define I2C1_BASE_ADDR (APB1_BASE_ADDR + 0x54u)
#define I2C2_BASE_ADDR (APB1_BASE_ADDR + 0x58u)
#define I2C3_BASE_ADDR (APB1_BASE_ADDR + 0x5cu)

#define SPI2_BASE_ADDR (APB1_BASE_ADDR + 0x38u)
#define SPI3_BASE_ADDR (APB1_BASE_ADDR + 0x3cu)

#define USART2_BASE_ADDR (APB1_BASE_ADDR + 0x44u)
#define USART3_BASE_ADDR (APB1_BASE_ADDR + 0x48u)

#define UART4_BASE_ADDR (APB1_BASE_ADDR + 0x4Cu)
#define UART5_BASE_ADDR (APB1_BASE_ADDR + 0x50u)

/*
 * APB2 peripheral base addresses
 */

#define EXT1_BASE_ADDR (APB2_BASE_ADDR + 0x3cu)
#define SPI1_BASE_ADDR (APB2_BASE_ADDR + 0x30u)
#define SYSCFG_BASE_ADDR (APB2_BASE_ADDR + 0x38u)
#define USART1_BASE_ADDR (APB2_BASE_ADDR + 0x10u)
#define USART6_BASE_ADDR (APB2_BASE_ADDR + 0x14u)

#define _reg volatile uint32_t

/*
 * Peripheral structure definitions
 */

typedef struct GPIO_RegDef_t {
	_reg MODER;
	_reg OTYPER;
	_reg OSPEEDR;
	_reg PUPDR;
	_reg IDR;
	_reg ODR;
	_reg BSRR;
	_reg LCKR;
	_reg AFR[2];
} GPIO_RegDef_t;

typedef struct RCC_RegDef_t {
	_reg RC;
	_reg PLLCFGR;
	_reg CFGR;
	_reg CIR;
	_reg AHB1RSTR;
	_reg AHB2RSTR;
	_reg AHB3RSTR;
	uint32_t _res1; // reserved
	_reg APB1RSTR;
	_reg APB2RSTR;
	uint32_t _res2[2]; // reserved
	_reg AHB1ENR;
	_reg AHB2ENR;
	_reg AHB3ENR;
	uint32_t __res3; // reserved
	_reg APB1ENR;
	_reg APB2ENR;
	uint32_t __res4[2]; // reserved
	_reg AHB1LPENR;
	_reg AHB2LPENR;
	_reg AHB3LPENR;
	uint32_t __res5; // reserved
	_reg APB1LPENR;
	_reg APB2LPENR;
	uint32_t __res6[2]; // reserved
	_reg BCDR;
	_reg CSR;
	uint32_t __res7[2];
	_reg SSCGR;
	_reg PLLI2CFGR;
} RCC_RegDef_t;

/*
 * Peripheral definitions
 */

#define GPIOA ((GPIO_RegDef_t*)GPIOA_BASE_ADDR)
#define GPIOB ((GPIO_RegDef_t*)GPIOB_BASE_ADDR)
#define GPIOC ((GPIO_RegDef_t*)GPIOC_BASE_ADDR)
#define GPIOD ((GPIO_RegDef_t*)GPIOD_BASE_ADDR)
#define GPIOE ((GPIO_RegDef_t*)GPIOE_BASE_ADDR)
#define GPIOF ((GPIO_RegDef_t*)GPIOF_BASE_ADDR)
#define GPIOG ((GPIO_RegDef_t*)GPIOG_BASE_ADDR)
#define GPIOH ((GPIO_RegDef_t*)GPIOH_BASE_ADDR)
#define GPIOI ((GPIO_RegDef_t*)GPIOI_BASE_ADDR)

#define RCC ((RCC_RegDef_t*)RCC_BASE_ADDR)

/*
 * Clock enable macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN() (RCC->AHB1ENR |= (1 << 0x0u))
#define GPIOB_PCLK_EN() (RCC->AHB1ENR |= (1 << 0x1u))
#define GPIOC_PCLK_EN() (RCC->AHB1ENR |= (1 << 0x2u))
#define GPIOD_PCLK_EN() (RCC->AHB1ENR |= (1 << 0x3u))
#define GPIOE_PCLK_EN() (RCC->AHB1ENR |= (1 << 0x4u))
#define GPIOF_PCLK_EN() (RCC->AHB1ENR |= (1 << 0x5u))
#define GPIOG_PCLK_EN() (RCC->AHB1ENR |= (1 << 0x6u))
#define GPIOH_PCLK_EN() (RCC->AHB1ENR |= (1 << 0x7u))
#define GPIOI_PCLK_EN() (RCC->AHB1ENR |= (1 << 0x8u))

/*
 * Clock enable macros for I2Cx peripherals
 */

#define I2C1_PCLK_EN() (RCC->APB1ENR |= (1 << 0x15u))
#define I2C2_PCLK_EN() (RCC->APB1ENR |= (1 << 0x16u))
#define I2C3_PCLK_EN() (RCC->APB1ENR |= (1 << 0x17u))

/*
 * Clock enable macros for SPIx peripherals
 */

#define SPI1_PCLK_EN() (RCC->APB2ENR |= (1 << 0xcu))
#define SPI2_PCLK_EN() (RCC->APB1ENR |= (1 << 0xeu))
#define SPI3_PCLK_EN() (RCC->APB1ENR |= (1 << 0xfu))

/*
 * Clock enable macros for USARTx peripherals
 */

#define USART1_PCLK_EN() (RCC->APB2ENR |= (1 << 0x4u))
#define USART2_PCLK_EN() (RCC->APB1ENR |= (1 << 0x11u))
#define USART3_PCLK_EN() (RCC->APB1ENR |= (1 << 0x12u))

/*
 * Clock enable macros for SYSCFG peripherals
 */

#define SYSCFG_PCLK_EN() (RCC->APB2ENR |= (1 << 0xeu))

/*
 * Clock disable macros for GPIOx peripherals
 */

#define GPIOA_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 0x0u))
#define GPIOB_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 0x1u))
#define GPIOC_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 0x2u))
#define GPIOD_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 0x3u))
#define GPIOE_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 0x4u))
#define GPIOF_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 0x5u))
#define GPIOG_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 0x6u))
#define GPIOH_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 0x7u))
#define GPIOI_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 0x8u))

/*
 * Clock disable macros for I2Cx peripherals
 */

#define I2C1_PCLK_DI() (RCC->APB1ENR &= ~(1 << 0x15u))
#define I2C2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 0x16u))
#define I2C3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 0x17u))

/*
 * Clock disable macros for SPIx peripherals
 */

#define SPI1_PCLK_DI() (RCC->APB2ENR &= ~(1 << 0xcu))
#define SPI2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 0xeu))
#define SPI3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 0xfu))

/*
 * Clock disable macros for USARTx peripherals
 */

#define USART1_PCLK_DI() (RCC->APB2ENR &= ~(1 << 0x4u))
#define USART2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 0x11u))
#define USART3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 0x12u))

/*
 * Clock disable macros for SYSCFG peripherals
 */

#define SYSCFG_PCLK_DI() (RCC->APB2ENR &= ~(1 << 0xeu))

/*
 * REgister reset macros
 */
// use trick to execute multiple c statements in one macro; "do while condition zero"
#define GPIOA_REG_RESET() do { (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0));} while(0)
#define GPIOB_REG_RESET() do { (RCC->AHB1RSTR |= (1 << 0x1)); (RCC->AHB1RSTR &= ~(1 << 0x1));} while(0)
#define GPIOC_REG_RESET() do { (RCC->AHB1RSTR |= (1 << 0x2)); (RCC->AHB1RSTR &= ~(1 << 0x2));} while(0)
#define GPIOD_REG_RESET() do { (RCC->AHB1RSTR |= (1 << 0x3)); (RCC->AHB1RSTR &= ~(1 << 0x3));} while(0)
#define GPIOE_REG_RESET() do { (RCC->AHB1RSTR |= (1 << 0x4)); (RCC->AHB1RSTR &= ~(1 << 0x4));} while(0)
#define GPIOF_REG_RESET() do { (RCC->AHB1RSTR |= (1 << 0x5)); (RCC->AHB1RSTR &= ~(1 << 0x5));} while(0)
#define GPIOG_REG_RESET() do { (RCC->AHB1RSTR |= (1 << 0x6)); (RCC->AHB1RSTR &= ~(1 << 0x6));} while(0)
#define GPIOH_REG_RESET() do { (RCC->AHB1RSTR |= (1 << 0x7)); (RCC->AHB1RSTR &= ~(1 << 0x7));} while(0)
#define GPIOI_REG_RESET() do { (RCC->AHB1RSTR |= (1 << 0x8)); (RCC->AHB1RSTR &= ~(1 << 0x8));} while(0)

// Generic macros
#define ENABLE 1
#define DISABLE 0
#define SET ENABLE
#define RESET DISABLE

#endif /* INC_STM32F407XX_H_ */
