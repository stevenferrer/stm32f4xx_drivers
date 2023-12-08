/*
 * stm32f407xx.h
 *
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

/*
 * Memory base addresses
 */
#define FLASH_BASE_ADDR 0x08000000U // main memory base address
#define SRAM1_BASE_ADDR 0x20000000U // sram1 base address
#define SRAM2_BASE_ADDR 0x2001C000U // sram2 base address
#define ROM_BASE_ADDR 0x1fff0000U // system memory base address
#define SRAM SRAM1_BASE_ADDR

/*
 * AHBx and APBx base addresses
 */
#define PERI_BASE_ADDR 0x40000000U
#define APB1_BASE_ADDR PERI_BASE_ADDR
#define APB2_BASE_ADDR 0x40010000U
#define AHB1_BASE_ADDR 0x40020000U
#define AHB2_BASE_ADDR 0x50000000U

/*
 * AHB1 peripheral base addresses
 */

#define GPIOA_BASE_ADDR (AHB1_BASE_ADDR + 0x00U)
#define GPIOB_BASE_ADDR (AHB1_BASE_ADDR + 0x04U)
#define GPIOC_BASE_ADDR (AHB1_BASE_ADDR + 0x08U)
#define GPIOD_BASE_ADDR (AHB1_BASE_ADDR + 0x0cU)
#define GPIOE_BASE_ADDR (AHB1_BASE_ADDR + 0x10U)
#define GPIOF_BASE_ADDR (AHB1_BASE_ADDR + 0x14U)
#define GPIOG_BASE_ADDR (AHB1_BASE_ADDR + 0x18U)
#define GPIOH_BASE_ADDR (AHB1_BASE_ADDR + 0x1cU)
#define GPIOI_BASE_ADDR (AHB1_BASE_ADDR + 0x20U)

/*
 * APB1 peripheral base addresses
 */

#define I2C1_BASE_ADDR (APB1_BASE_ADDR + 0x54U)
#define I2C2_BASE_ADDR (APB1_BASE_ADDR + 0x58U)
#define I2C3_BASE_ADDR (APB1_BASE_ADDR + 0x5cU)

#define SPI2_BASE_ADDR (APB1_BASE_ADDR + 0x38U)
#define SPI3_BASE_ADDR (APB1_BASE_ADDR + 0x3cU)

#define USART2_BASE_ADDR (APB1_BASE_ADDR + 0x44U)
#define USART3_BASE_ADDR (APB1_BASE_ADDR + 0x48U)

#define UART4_BASE_ADDR (APB1_BASE_ADDR + 0x4CU)
#define UART5_BASE_ADDR (APB1_BASE_ADDR + 0x50U)

/*
 * APB2 peripheral base addresses
 */

#define EXT1_BASE_ADDR (APB2_BASE_ADDR + 0x3cU)
#define SPI1_BASE_ADDR (APB2_BASE_ADDR + 0x30U)
#define SYSCFG_BASE_ADDR (APB2_BASE_ADDR + 0x38U)
#define USART1_BASE_ADDR (APB2_BASE_ADDR + 0x10U)
#define USART6_BASE_ADDR (APB2_BASE_ADDR + 0x14U)


#define _vo volatile

/*
 * Peripheral structure definitions
 */


typedef struct GPIO_RegDef_t {
	_vo uint32_t MODER;
	_vo uint32_t OTYPER;
	_vo uint32_t OSPEEDR;
	_vo uint32_t PUPDR;
	_vo uint32_t IDR;
	_vo uint32_t ODR;
	_vo uint32_t BSRR;
	_vo uint32_t LCKR;
	_vo uint32_t AFR[2];
} GPIO_RegDef_t;

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


#endif /* INC_STM32F407XX_H_ */
