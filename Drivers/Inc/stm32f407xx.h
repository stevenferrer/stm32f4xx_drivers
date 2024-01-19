/*
 * stm32f407xx.h
 *
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

#define _reg volatile uint32_t

/*
 * Processor specific register
 */

/*
 * ARM cortex-m4 NVIC ISERx register addr
 */
#define NVIC_ISER0 ((_reg*)0xe000e100)
#define NVIC_ISER1 ((_reg*)0xe000e104)
#define NVIC_ISER2 ((_reg*)0xe000e108)
#define NVIC_ISER3 ((_reg*)0xe000e10c)

/*
 * ARM cortex-m4 NVIC ICERx register addr
 */
#define NVIC_ICER0 ((_reg*)0xe000e180)
#define NVIC_ICER1 ((_reg*)0xe000e184)
#define NVIC_ICER2 ((_reg*)0xe000e188)
#define NVIC_ICER3 ((_reg*)0xe000e18c)

/*
 * ARM cortex-m4 NVIC IPRx register addr
 */
#define NVIC_IPR_BASE_ADDR ((_reg*)0xe000e400)

#define NO_PRIORITY_BITS_IMPLEMENTED 4

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

#define GPIOA_BASE_ADDR (AHB1_BASE_ADDR + 0x0000u)
#define GPIOB_BASE_ADDR (AHB1_BASE_ADDR + 0x0400u)
#define GPIOC_BASE_ADDR (AHB1_BASE_ADDR + 0x0800u)
#define GPIOD_BASE_ADDR (AHB1_BASE_ADDR + 0x0c00u)
#define GPIOE_BASE_ADDR (AHB1_BASE_ADDR + 0x1000u)
#define GPIOF_BASE_ADDR (AHB1_BASE_ADDR + 0x1400u)
#define GPIOG_BASE_ADDR (AHB1_BASE_ADDR + 0x1800u)
#define GPIOH_BASE_ADDR (AHB1_BASE_ADDR + 0x1c00u)
#define GPIOI_BASE_ADDR (AHB1_BASE_ADDR + 0x2000u)

#define RCC_BASE_ADDR (AHB1_BASE_ADDR + 0x3800u)

/*
 * APB1 peripheral base addresses
 */

#define I2C1_BASE_ADDR (APB1_BASE_ADDR + 0x5400u)
#define I2C2_BASE_ADDR (APB1_BASE_ADDR + 0x5800u)
#define I2C3_BASE_ADDR (APB1_BASE_ADDR + 0x5c00u)

#define SPI2_BASE_ADDR (APB1_BASE_ADDR + 0x3800u)
#define SPI3_BASE_ADDR (APB1_BASE_ADDR + 0x3c00u)

#define USART2_BASE_ADDR (APB1_BASE_ADDR + 0x4400u)
#define USART3_BASE_ADDR (APB1_BASE_ADDR + 0x4800u)

#define UART4_BASE_ADDR (APB1_BASE_ADDR + 0x4c00u)
#define UART5_BASE_ADDR (APB1_BASE_ADDR + 0x5000u)

/*
 * APB2 peripheral base addresses
 */

#define EXT1_BASE_ADDR (APB2_BASE_ADDR + 0x3c00u)
#define SPI1_BASE_ADDR (APB2_BASE_ADDR + 0x3000u)
#define SYSCFG_BASE_ADDR (APB2_BASE_ADDR + 0x3800u)
#define USART1_BASE_ADDR (APB2_BASE_ADDR + 0x1000u)
#define USART6_BASE_ADDR (APB2_BASE_ADDR + 0x1400u)

/*
 * Peripheral structure definitions
 */

// See GPIO register map
typedef struct GPIO_RegDef_t {
	_reg MODER; // mode register
	_reg OTYPER; // output type register
	_reg OSPEEDR; // output speed register
	_reg PUPDR; // pull-up/pull-down register
	_reg IDR; // input data register
	_reg ODR; // output data register
	_reg BSRR; // bit set/reset register
	_reg LCKR; // configuration lock register
	_reg AFR[2]; // alternate function register
} GPIO_RegDef_t;

// See RCC register map
typedef struct RCC_RegDef_t {
	_reg CR;
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

// See EXTI register map
typedef struct EXTI_RegDef_t {
	_reg IMR;
	_reg EMR;
	_reg RTSR;
	_reg FTSR;
	_reg SWIER;
	_reg PR;
} EXTI_RegDef_t;

// See SYSCFG register map
typedef struct SYSCFG_RegDef_t {
	_reg MEMRMP;
	_reg PMC;
	_reg EXTICR[4]; // 0x08 to 0x14
	// registers for 0x18 and 0x1c not mentioned in datasheet
	uint32_t __res1[2];
	_reg CMPCR; // 0x20
} SYSCFG_RegDef_t;

// See SPI register map
typedef struct SPI_RegDef_t {
	_reg CR1;
	_reg CR2;
	_reg SR;
	_reg DR;
	_reg CRCPR;
	_reg RXCRCR;
	_reg TXCRCR;
	_reg I2SCFGR;
	_reg I2SPR;
} SPI_RegDef_t;

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

#define EXTI ((EXTI_RegDef_t*)EXT1_BASE_ADDR)

#define SYSCFG ((SYSCFG_RegDef_t*)SYSCFG_BASE_ADDR)

#define SPI1 ((SPI_RegDef_t*)SPI1_BASE_ADDR)
#define SPI2 ((SPI_RegDef_t*)SPI2_BASE_ADDR)
#define SPI3 ((SPI_RegDef_t*)SPI3_BASE_ADDR)

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
 * Register reset macros. Works by setting the reset register to 1 and then back to zero.
 * Use "do while condition zero" to execute multiple statements.
 */
#define GPIOA_REG_RESET() do { (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); } while(0)
#define GPIOB_REG_RESET() do { (RCC->AHB1RSTR |= (1 << 0x1)); (RCC->AHB1RSTR &= ~(1 << 0x1)); } while(0)
#define GPIOC_REG_RESET() do { (RCC->AHB1RSTR |= (1 << 0x2)); (RCC->AHB1RSTR &= ~(1 << 0x2)); } while(0)
#define GPIOD_REG_RESET() do { (RCC->AHB1RSTR |= (1 << 0x3)); (RCC->AHB1RSTR &= ~(1 << 0x3)); } while(0)
#define GPIOE_REG_RESET() do { (RCC->AHB1RSTR |= (1 << 0x4)); (RCC->AHB1RSTR &= ~(1 << 0x4)); } while(0)
#define GPIOF_REG_RESET() do { (RCC->AHB1RSTR |= (1 << 0x5)); (RCC->AHB1RSTR &= ~(1 << 0x5)); } while(0)
#define GPIOG_REG_RESET() do { (RCC->AHB1RSTR |= (1 << 0x6)); (RCC->AHB1RSTR &= ~(1 << 0x6)); } while(0)
#define GPIOH_REG_RESET() do { (RCC->AHB1RSTR |= (1 << 0x7)); (RCC->AHB1RSTR &= ~(1 << 0x7)); } while(0)
#define GPIOI_REG_RESET() do { (RCC->AHB1RSTR |= (1 << 0x8)); (RCC->AHB1RSTR &= ~(1 << 0x8)); } while(0)

#define SPI1_REG_RESET() do { RCC->APB2RSTR |= (1 << 0xc); (RCC->APB2RSTR &= ~(1<<0xc)); } while(0)
#define SPI2_REG_RESET() do { RCC->APB1RSTR |= (1 << 0xe); (RCC->APB1RSTR &= ~(1<<0xe)); } while(0)
#define SPI3_REG_RESET() do { RCC->APB1RSTR |= (1 << 0xf); (RCC->APB1RSTR &= ~(1<<0xf)); } while(0)

#define GPIO_BASE_ADDR_TO_PORT_CODE(x) ((x==GPIOA) ? 0 : \
							(x == GPIOB) ? 1 : \
							(x == GPIOC) ? 2 : \
							(x == GPIOD) ? 3 : \
							(x == GPIOE) ? 4 : \
							(x == GPIOF) ? 5 : \
							(x == GPIOG) ? 6 : \
							(x == GPIOH) ? 7 : \
							(x == GPIOI) ? 8 : 0)

// IRQ (interrupt request numbers)
#define IRQ_NO_EXTI0 6
#define IRQ_NO_EXTI1 7
#define IRQ_NO_EXTI2 8
#define IRQ_NO_EXTI3 9
#define IRQ_NO_EXTI4 10
#define IRQ_NO_EXTI9_5 23
#define IRQ_NO_EXTI15_10 40

#define NVIC_IRQ_PRIORITY_0 0
#define NVIC_IRQ_PRIORITY_15 15

// Generic macros
#define ENABLE 1
#define DISABLE 0
#define SET ENABLE
#define RESET DISABLE
#define FLAG_RESET RESET

/*
 * Bit position macros
 */
#define SPI_CR1_CPHA 0
#define SPI_CR1_CPOL 1
#define SPI_CR1_MSTR 2
#define SPI_CR1_BR 3
#define SPI_CR1_SSM 9
#define SPI_CR1_RXONLY 10
#define SPI_CR1_DFF 11
#define SPI_CR1_BIDIOE 14
#define SPI_CR1_BIDIMODE 15

#define SPI_SR_RXNE 0
#define SPI_SR_TXE 1
#define SPI_SR_CHSIDE 2
#define SPI_SR_CRCERR 3
#define SPI_SR_MODF 4
#define SPI_SR_OVR 5
#define SPI_SR_BSY 6
#define SPI_SR_FRE 7

#endif /* INC_STM32F407XX_H_ */
