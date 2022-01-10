/*
 * stm32f407xx_i2c_driver.h
 */

#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_

#include "stm32f407xx.h"

/*
 * This is a Configuration structure for a GPIO pin
 */

typedef struct
{
	uint32_t I2C_SCLSpeed;
	uint8_t I2C_DeviceAddress;
	uint8_t I2C_ACKControl;
	uint16_t I2C_FMDutyCycle;
}I2C_Config_t;

/*
 * This is a Handle structure for a I2C pin
 */

typedef struct
{
	// pointer to hold the base address of the I2C peripheral
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;
}I2C_Handle_t;

/*
 * Possible I2C Application states
 */

#define I2C_READY			0
#define I2C_BUSY_IN_RX 		1
#define I2C_BUSY_IN_TX 		2

/*
 *  @I2C_SCLSpeed
 */

#define I2C_SCL_SPEED_SM 	100000
#define I2C_SCL_SPEED_FM2K	200000
#define I2C_SCL_SPEED_FM4K	400000

/*
 * @I2C_ACKControl
 */

#define I2C_ACK_ENABLE		1
#define I2C_ACK_DISABLE		0

/*
 * @I2C_FMDutyCycle
 */

#define I2C_FM_DUTY_2		0
#define I2C_FM_DUTY_16_9	1

/*
 * I2C related status flag definitions
 */
#define I2C_FLAG_TXE 		( 1 << I2C_SR1_TXE )
#define I2C_FLAG_RXNE 		( 1 << I2C_SR1_RXNE )
#define I2C_FLAG_SB			( 1 << I2C_SR1_SB )
#define I2C_FLAG_ADDR		( 1 << I2C_SR1_ADDR )
#define I2C_FLAG_BTF		( 1 << I2C_SR1_BTF )
#define I2C_FLAG_STOPF		( 1 << I2C_SR1_STOPF )
#define I2C_FLAG_BERR		( 1 << I2C_SR1_BERR )
#define I2C_FLAG_ARLO		( 1 << I2C_SR1_ARLO )
#define I2C_FLAG_AF			( 1 << I2C_SR1_AF )
#define I2C_FLAG_OVR		( 1 << I2C_SR1_OVR )
#define I2C_FLAG_TIMEOUT	( 1 << I2C_SR1_TIMEOUT )

#define I2C_DISABLE_SR  	RESET
#define I2C_ENABLE_SR   	SET

// APIs supported by this driver //

/*
 * Peripheral Clock Setup
 */

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/*
 * Init and De-Init
 */

void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/*
 * Data send and receive
 */

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer,uint32_t Len,uint8_t SlaveAddr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer,uint8_t Len,uint8_t SlaveAddr);
void I2C_SlaveSendData(void);
uint8_t I2C_SlaveReceiveData(void);

/*
 * Other Peripheral Control APIs
 */

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx , uint32_t FlagName);
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

#endif /* INC_STM32F407XX_I2C_DRIVER_H_ */
