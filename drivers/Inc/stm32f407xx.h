/*
 * stm32f407xx.h
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_


#include <stdint.h>


#define _vo volatile


/*
 * Base addresses of flash and SRAM memories
 */

#define FLASH_BASEADDR			0x08000000U
#define SRAM1_BASEADDR			0x20000000U
#define SRAM2_BASEADDR			0x20001C00U
#define ROM_BASEADDR			0x1FFF0000U
#define SRAM_BASEADDR			SRAM1_BASEADDR

/*
 * AHBx and APBx Bus Peripheral base addresses
 */

#define PERIPH_BASE				0x40000000U
#define APB1PERIPH_BASE			PERIPH_BASE
#define APB2PERIPH_BASE			0x40010000U
#define AHB1PERIPH_BASE			0x40020000U
#define AHB2PERIPH_BASE			0x50000000U

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 */

#define GPIOA_BASEADDR			(AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASEADDR			(AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR			(AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR			(AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASEADDR			(AHB1PERIPH_BASE + 0x1000)
#define GPIOF_BASEADDR			(AHB1PERIPH_BASE + 0x1400)
#define GPIOG_BASEADDR			(AHB1PERIPH_BASE + 0x1800)
#define GPIOH_BASEADDR			(AHB1PERIPH_BASE + 0x1C00)
#define GPIOI_BASEADDR			(AHB1PERIPH_BASE + 0x2000)
#define RCC_BASEADDR			(AHB1PERIPH_BASE + 0x3800)
#define FIR_BASEADDR			(AHB1PERIPH_BASE + 0x3C00)
#define DMA1_BASEADDR			(AHB1PERIPH_BASE + 0x6000)
#define DMA2_BASEADDR			(AHB1PERIPH_BASE + 0x6400)


/*
 * Base addresses of peripherals which are hanging on APB1 bus
 */

#define I2C1_BASEADDR			(APB1PERIPH_BASE + 0x5400)
#define I2C2_BASEADDR			(APB1PERIPH_BASE + 0x5800)
#define I2C3_BASEADDR			(APB1PERIPH_BASE + 0x5C00)

#define SPI2_BASEADDR			(APB1PERIPH_BASE + 0x3800)
#define SPI3_BASEADDR			(APB1PERIPH_BASE + 0x3C00)

#define USART2_BASEADDR			(APB1PERIPH_BASE + 0x4400)
#define USART3_BASEADDR			(APB1PERIPH_BASE + 0x4800)
#define UART4_BASEADDR			(APB1PERIPH_BASE + 0x4C00)
#define UART5_BASEADDR			(APB1PERIPH_BASE + 0x5000)

/*
 * Base addresses of peripherals which are hanging on APB2 bus
 */

#define SPI1_BASEADDR			(APB2PERIPH_BASE + 0x3000)
#define USART1_BASEADDR			(APB2PERIPH_BASE + 0x1000)
#define USART6_BASEADDR			(APB2PERIPH_BASE + 0x1400)
#define EXTI_BASEADDR			(APB2PERIPH_BASE + 0x3C00)
#define SYSCFG_BASEADDR			(APB2PERIPH_BASE + 0x3800)

/*
 * Peripheral register definition structure for GPIO
 */

typedef struct
{
	_vo	uint32_t MODER;				// GPIO port mode register, 						Address offset : 0x00//
	_vo uint32_t OTYPER;			// GPIO port output type register, 					Address offset : 0x04//
	_vo	uint32_t OSPEEDR;			// GPIO port output speed register, 				Address offset : 0x08//
	_vo	uint32_t PUPDR;				// GPIO port pull-up/down register, 				Address offset : 0x0C//
	_vo	uint32_t IDR;				// GPIO port input data register, 					Address offset : 0x10//
	_vo	uint32_t ODR;				// GPIO port output data register, 					Address offset : 0x14//
	_vo	uint32_t BSRR;				// GPIO port bit set/reset register, 				Address offset : 0x18//
	_vo	uint32_t LCKR;				// GPIO port configuration lock register,			Address offset : 0x1C//
	_vo	uint32_t AFR[2];			/* AFR[0]: GPIO alternate function low register
								   	   AFR[1]: GPIO alternate function high register,	Address offset : 0x20-24 */
}GPIO_RegDef_t;

/*
 * Peripheral register definition structure for RCC
 */

typedef struct
{
	_vo uint32_t CR;
	_vo uint32_t PLLCFGR;
	_vo uint32_t CFGR;
	_vo uint32_t CIR;
	_vo uint32_t AHB1RSTR;
	_vo uint32_t AHB2RSTR;
	_vo uint32_t AHB3RSTR;
		uint32_t RESERVED0;
	_vo uint32_t APB1RSTR;
	_vo uint32_t APB2RSTR;
		uint32_t RESERVED1[2];
	_vo uint32_t AHB1ENR;
	_vo uint32_t AHB2ENR;
	_vo uint32_t AHB3ENR;
		uint32_t RESERVED3;
	_vo uint32_t APB1ENR;
	_vo uint32_t APB2ENR;
		uint32_t RESERVED4[2];
	_vo uint32_t AHB1LPENR;
	_vo uint32_t AHB2LPENR;
	_vo uint32_t AHB3LPENR;
		uint32_t RESERVED6;
	_vo uint32_t APB1LPENR;
	_vo uint32_t APB2LPENR;
		uint32_t RESERVED7[2];
	_vo uint32_t BDCR;
	_vo uint32_t CSR;
		uint32_t RESERVED8[2];
	_vo uint32_t SSCGR;
	_vo uint32_t PLLI2SCFGR;
}RCC_RegDef_t;

/*
 * Peripheral register definition structure for EXTI
 */

typedef struct
{
	_vo uint32_t IMR;					// Address offset : 0x00 //
	_vo uint32_t EMR;					// Address offset : 0x04 //
	_vo uint32_t RTSR;					// Address offset : 0x08 //
	_vo uint32_t FTSR;					// Address offset : 0x0C //
	_vo uint32_t SWIER;					// Address offset : 0x10 //
	_vo uint32_t PR;					// Address offset : 0x14 //
}EXTI_RegDef_t;

/*
 * Peripheral register definition structure for SYSCFG
 */

typedef struct
{
	_vo uint32_t MEMRMP;
	_vo uint32_t PMC;
	_vo uint32_t EXTICR[4];
		uint32_t RESERVED[2];
	_vo uint32_t CMPCR;
}SYSCFG_RegDef_t;

/*
 * Peripheral register definition structure for SPI
 */

typedef struct
{
	_vo uint32_t CR1;				// Address offset : 0x00 //
	_vo uint32_t CR2;				// Address offset : 0x04 //
	_vo uint32_t SR;				// Address offset : 0x08 //
	_vo uint32_t DR;				// Address offset : 0x0C //
	_vo uint32_t CRCPR;				// Address offset : 0x10 //
	_vo uint32_t RXCRCR;			// Address offset : 0x14 //
	_vo uint32_t TXCRCR;			// Address offset : 0x18 //
	_vo uint32_t I2SCFGR;			// Address offset : 0x1C //
	_vo uint32_t I2SPR;				// Address offset : 0x20 //
}SPI_RegDef_t;

/*
 * Peripheral register definition structure for I2C
 */

typedef struct
{
	_vo uint32_t CR1;				// Address offset : 0x00 //
	_vo uint32_t CR2;				// Address offset : 0x04 //
	_vo uint32_t OAR1;				// Address offset : 0x08 //
	_vo uint32_t OAR2;				// Address offset : 0x0C //
	_vo uint32_t DR;				// Address offset : 0x10 //
	_vo uint32_t SR1;				// Address offset : 0x14 //
	_vo uint32_t SR2;				// Address offset : 0x18 //
	_vo uint32_t CCR;				// Address offset : 0x1C //
	_vo uint32_t TRISE;				// Address offset : 0x20 //
	_vo uint32_t FLTR;				// Address offset : 0x24 //
}I2C_RegDef_t;

/*
 * Peripheral register definition structure for USART
 */

typedef struct
{
	_vo uint32_t SR;				// Address offset : 0x00 //
	_vo uint32_t DR;				// Address offset : 0x04 //
	_vo uint32_t BRR;				// Address offset : 0x08 //
	_vo uint32_t CR1;				// Address offset : 0x0C //
	_vo uint32_t CR2;				// Address offset : 0x10 //
	_vo uint32_t CR3;				// Address offset : 0x14 //
	_vo uint32_t GTPR;				// Address offset : 0x18 //
}USART_RegDef_t;

/*
 * Peripheral definitions (Peripheral base addresses typecasted to xxx_RegDef_t)
 */

#define GPIOA						((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB						((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC						((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD						((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE						((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF						((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG						((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH						((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI						((GPIO_RegDef_t*)GPIOI_BASEADDR)
#define RCC							((RCC_RegDef_t*)RCC_BASEADDR)

#define RCC							((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI						((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG						((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1						((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2						((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3						((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4						((SPI_RegDef_t*)SPI4_BASEADDR)

#define I2C1						((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2						((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3						((I2C_RegDef_t*)I2C3_BASEADDR)

#define USART1						((USART_RegDef_t*)USART1_BASEADDR)
#define USART2						((USART_RegDef_t*)USART2_BASEADDR)
#define USART3						((USART_RegDef_t*)USART3_BASEADDR)
#define UART4						((USART_RegDef_t*)UART4_BASEADDR)
#define UART5						((USART_RegDef_t*)UART5_BASEADDR)
#define USART6						((USART_RegDef_t*)USART6_BASEADDR)

/*
 * Clock Enable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()				(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()				(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()				(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()				(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()				(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()				(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()				(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()				(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()				(RCC->AHB1ENR |= (1 << 8))

/*
 * Clock Enable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_EN()				(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()				(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()				(RCC->APB1ENR |= (1 << 23))

/*
 * Clock Enable Macros for SPIx peripherals
 */

#define SPI1_PCLK_EN()				(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()				(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()				(RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()				(RCC->APB2ENR |= (1 << 13))

/*
 * Clock Enable Macros for USARTx peripherals
 */

#define USART1_PCLK_EN()			(RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()			(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()			(RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()				(RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()				(RCC->APB1ENR |= (1 << 20))
#define USART6_PCLK_EN()			(RCC->APB2ENR |= (1 << 5))

/*
 * Clock Enable Macros for SYSCFG peripherals
 */

#define SYSCFG_PCLK_EN()			(RCC->APB2ENR |= (1 << 14))

/*
 * Clock Disable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 8))

/*
 * Clock Disable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 23))

/*
 * Clock Disable Macros for SPIx peripherals
 */

#define SPI1_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 13))

/*
 * Clock Disable Macros for USARTx peripherals
 */

#define USART1_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 5))

/*
 * Clock Disable Macros for SYSCFG peripherals
 */

#define SYSCFG_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 14))

/*
 * Macros to reset GPIOx peripherals
 */

#define GPIOA_REG_RESET()			do{ (RCC->AHB1RSTR |= (1<<0)); (RCC->AHB1RSTR &= ~(1<<0)); }while(0)
#define GPIOB_REG_RESET()			do{ (RCC->AHB1RSTR |= (1<<1)); (RCC->AHB1RSTR &= ~(1<<1)); }while(0)
#define GPIOC_REG_RESET()			do{ (RCC->AHB1RSTR |= (1<<2)); (RCC->AHB1RSTR &= ~(1<<2)); }while(0)
#define GPIOD_REG_RESET()			do{ (RCC->AHB1RSTR |= (1<<3)); (RCC->AHB1RSTR &= ~(1<<3)); }while(0)
#define GPIOE_REG_RESET()			do{ (RCC->AHB1RSTR |= (1<<4)); (RCC->AHB1RSTR &= ~(1<<4)); }while(0)
#define GPIOF_REG_RESET()			do{ (RCC->AHB1RSTR |= (1<<5)); (RCC->AHB1RSTR &= ~(1<<5)); }while(0)
#define GPIOG_REG_RESET()			do{ (RCC->AHB1RSTR |= (1<<6)); (RCC->AHB1RSTR &= ~(1<<6)); }while(0)
#define GPIOH_REG_RESET()			do{ (RCC->AHB1RSTR |= (1<<7)); (RCC->AHB1RSTR &= ~(1<<7)); }while(0)
#define GPIOI_REG_RESET()			do{ (RCC->AHB1RSTR |= (1<<8)); (RCC->AHB1RSTR &= ~(1<<8)); }while(0)

/*
 *  returns port code for given GPIOx base address
 */

/*
 * This macro returns a code( between 0 to 7) for a given GPIO base address(x)
 */

#define GPIO_BASEADDR_TO_CODE(x)      ( (x == GPIOA)?0:\
										(x == GPIOB)?1:\
										(x == GPIOC)?2:\
										(x == GPIOD)?3:\
								        (x == GPIOE)?4:\
								        (x == GPIOF)?5:\
								        (x == GPIOG)?6:\
								        (x == GPIOH)?7: \
								        (x == GPIOI)?8:0)

/*
 * Macros to reset SPIx peripherals
 */

#define SPI1_REG_RESET()			do{ (RCC->APB2RSTR |= (1<<12)); (RCC->APB2RSTR &= ~(1<<12)); }while(0)
#define SPI2_REG_RESET()			do{ (RCC->APB1RSTR |= (1<<14)); (RCC->APB1RSTR &= ~(1<<14)); }while(0)
#define SPI3_REG_RESET()			do{ (RCC->APB1RSTR |= (1<<15)); (RCC->APB1RSTR &= ~(1<<15)); }while(0)

/*
 * Macros to reset I2Cx peripherals
 */

#define I2C1_REG_RESET()			do{ (RCC->APB1RSTR |= (1<<21)); (RCC->APB1RSTR &= ~(1<<21)); }while(0)
#define I2C2_REG_RESET()			do{ (RCC->APB1RSTR |= (1<<22)); (RCC->APB1RSTR &= ~(1<<22)); }while(0)
#define I2C3_REG_RESET()			do{ (RCC->APB1RSTR |= (1<<23)); (RCC->APB1RSTR &= ~(1<<23)); }while(0)

/*
 * Macros to reset USARTx peripherals
 */

#define USART1_REG_RESET()			do{	(RCC->APB2RSTR |= (1<<4));	(RCC->APB2RSTR &= ~(1<<4));	}while(0)
#define USART2_REG_RESET()			do{	(RCC->APB1RSTR |= (1<<17));	(RCC->APB1RSTR &= ~(1<<17));}while(0)
#define USART3_REG_RESET()			do{	(RCC->APB1RSTR |= (1<<18));	(RCC->APB1RSTR &= ~(1<<18));}while(0)
#define UART4_REG_RESET()			do{	(RCC->APB1RSTR |= (1<<19));	(RCC->APB1RSTR &= ~(1<<19));}while(0)
#define UART5_REG_RESET()			do{	(RCC->APB1RSTR |= (1<<20));	(RCC->APB1RSTR &= ~(1<<20));}while(0)
#define USART6_REG_RESET()			do{	(RCC->APB2RSTR |= (1<<5));	(RCC->APB2RSTR &= ~(1<<5));	}while(0)

/*
 * Some gereric marcos
 */

#define ENABLE 						1
#define DISABLE						0
#define SET							ENABLE
#define RESET						DISABLE
#define FLAG_RESET         			RESET
#define FLAG_SET 					SET

// Bit position definitions of I2C peripheral

/*
 * Bit position definitions I2C_CR1
 */

#define I2C_CR1_PE	 		0
#define I2C_CR1_NOSTRECH	7
#define I2C_CR1_START		8
#define I2C_CR1_STOP		9
#define I2C_CR1_ACK			10
#define I2C_CR1_RESET		15

/*
 * Bit position definitions I2C_CR2
 */

#define I2C_CR2_FREQ	 	0
#define I2C_CR2_ITERREN	 	8
#define I2C_CR2_ITEVTEN		9
#define I2C_CR2_ITBUFEN		10

/*
 * Bit position definitions I2C_OAR1
 */

#define I2C_OAR1_ADD0	 	0
#define I2C_OAR1_ADD7_1	 	1
#define I2C_OAR1_ADD9_8	 	8
#define I2C_OAR1_ADDMODE	15

/*
 * Bit position definitions I2C_SR1
 */

#define I2C_SR1_SB			0
#define I2C_SR1_ADDR		1
#define I2C_SR1_BTF			2
#define I2C_SR1_ADD10		3
#define I2C_SR1_STOPF		4
#define I2C_SR1_RXNE		6
#define I2C_SR1_TXE			7
#define I2C_SR1_BERR		8
#define I2C_SR1_ARLO		9
#define I2C_SR1_AF			10
#define I2C_SR1_OVR			11
#define I2C_SR1_TIMEOUT		14

/*
 * Bit position definitions I2C_SR2
 */

#define I2C_SR2_MSL			0
#define I2C_SR2_BUSY		1
#define I2C_SR2_TRA			2
#define I2C_SR2_GENCALL		4
#define I2C_SR2_DUALF		7

/*
 * Bit position definitions I2C_CCR
 */

#define I2C_CCR_CCR 		0
#define I2C_CCR_DUTY 		14
#define I2C_CCR_FS  		15

#include "stm32f407xx_rcc_driver.h"
#include "stm32f407xx_i2c_driver.h"

#endif /* INC_STM32F407XX_H_ */
