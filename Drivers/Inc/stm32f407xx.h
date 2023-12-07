/*
 * stm32f407xx.h
 *
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

/*
 * Memory base addresses
 */
#define FLASH_BASE_ADDR 0x0800_0000U // main memory base address
#define SRAM1_BASE_ADDR 0x2000_0000U // sram1 base address
#define SRAM2_BASE_ADDR 0x2001_C000U // sram2 base address
#define ROM_BASE_ADDR 0x1fff_0000U // system memory base address
#define SRAM SRAM1_BASE_ADDR

/*
 * AHBx and APBx base addresses
 */
#define PERI_BASE_ADDR 0x4000_0000U
#define APB1_BASE_ADDR PERI_BASE_ADDR
#define APB2_BASE_ADDR 0x4001_0000U
#define AHB1_BASE_ADDR 0x4002_0000U
#define AHB2_BASE_ADDR 0x5000_0000U

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


#endif /* INC_STM32F407XX_H_ */
