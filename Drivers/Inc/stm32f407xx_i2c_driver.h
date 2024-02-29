/*
 * stm32f407xx_i2c_driver.h
 *
 *  Created on: Feb 5, 2024
 *      Author: sf
 */

#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_

#include "stm32f407xx.h"

typedef struct I2C_Config_t {
	uint32_t SCLSpeed;
	uint8_t DeviceAddress;
	uint8_t ACKControl;
	uint16_t FMDutyCycle;
} I2C_Config_t;

typedef struct I2C_Handle_t {
	I2C_RegDef_t *i2cx;
	I2C_Config_t config;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer; // rx buffer
	uint32_t txLen; // tx len
	uint32_t rxLen; // rx len
	uint8_t state;  // communication state
	uint8_t devAddr; // slave/device address
	uint32_t rxSize; // rx size
	uint8_t sr; // repeated start
} I2C_Handle_t;

/*
 * i2c SCL speed
 */
#define I2C_SCL_SPEED_STD 100000
#define I2C_SCL_SPEED_FAST2K 200000
#define I2C_SCL_SPEED_FAST4K 400000

/*
 * i2c ack
 */
#define I2C_ACK_DISABLE 0
#define I2C_ACK_ENABLE 1

/*
 * i2c duty cycle
 */
#define I2C_FM_DUTY_2 0
#define I2C_FM_DUTY_16_9 1

/*
 * I2C related status flag defintions
 */

#define I2C_FLAG_SB (1 << I2C_SR1_SB)
#define I2C_FLAG_ADDR (1<< I2C_SR1_ADDR)
#define I2C_FLAG_BTF (1 << I2C_SR1_BTF)
#define I2C_FLAG_STOPF (1 << I2C_SR1_STOPF)
#define I2C_FLAG_RXNE (1 << I2C_SR1_RXNE)
#define I2C_FLAG_TXE (1 << I2C_SR1_TXE)
#define I2C_FLAG_BERR (1 << I2C_SR1_BERR)
#define I2C_FLAG_ARLO (1 << I2C_SR1_ARLO)
#define I2C_FLAG_AF (1 << I2C_SR1_AF)
#define I2C_FLAG_OVR (1 << I2C_SR1_OVR)
#define I2C_FLAG_TIMEOUT (1 << I2C_SR1_TIMEOUT)

/*
 * Read/write macros
 */
#define I2C_RW_WRITE 0
#define I2C_RW_READ 1

/*
 * Repeated start macros
 */
#define I2C_SR_DISABLE 0
#define I2C_SR_ENABLE 1

/*
 *  I2C communication state
 */
#define I2C_STATE_READY 0
#define I2C_STATE_BUSY_IN_RX 1
#define I2C_STATE_BUSY_IN_TX 2

/*
 * I2C application event macros
 */

#define I2C_EV_TX_CMPLT 0
#define I2C_EV_RX_CMPLT 1
#define I2C_EV_STOP 2

/*
 * I2C error macros
 */
#define I2C_ERROR_BERR  3
#define I2C_ERROR_ARLO  4
#define I2C_ERROR_AF    5
#define I2C_ERROR_OVR   6
#define I2C_ERROR_TIMEOUT 7

/*
 * Peripheral clock setup
 */
void I2C_PeriClockCtrl(I2C_RegDef_t *pI2Cx, uint8_t enable);

/*
 * Init and de-init
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer,
		uint32_t len, uint8_t slaveAddress, uint8_t sr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer,
		uint32_t len, uint8_t slaveAddress, uint8_t sr);

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer,
		uint32_t len, uint8_t slaveAddress, uint8_t sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer,
		uint32_t len, uint8_t slaveAddress, uint8_t sr);

/*
 * IRQ config and ISR handling
 */
void I2C_IRQInterruptConfig(uint8_t irqNumber, uint8_t enable);
void I2C_IRQPriorityConfig(uint8_t irqNumber, uint8_t irqPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);

void I2C_AppEventCallback(I2C_Handle_t *pI2Candle, uint8_t event);

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t enable);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t mask);
void I2C_EnableAcking(I2C_RegDef_t *pI2Cx, uint8_t enable);

void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);

#endif /* INC_STM32F407XX_I2C_DRIVER_H_ */
