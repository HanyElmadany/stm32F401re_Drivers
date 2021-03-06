/*
 * stm32f401xx.h
 *
 *  Created on: Jun 12, 2021
 *      Author: hany.elmadany
 */

#ifndef INC_STM32F401XX_H_
#define INC_STM32F401XX_H_
#include<stdint.h>

#define __vo                     volatile

/**********************************START:Processor Specific Details **********************************/
/*
 * ARM Cortex Mx Processor NVIC ISERx register Addresses
 */

#define NVIC_ISER0          ( (__vo uint32_t*)0xE000E100 )
#define NVIC_ISER1          ( (__vo uint32_t*)0xE000E104 )
#define NVIC_ISER2          ( (__vo uint32_t*)0xE000E108 )
#define NVIC_ISER3          ( (__vo uint32_t*)0xE000E10c )


/*
 * ARM Cortex Mx Processor NVIC ICERx register Addresses
 */
#define NVIC_ICER0 			((__vo uint32_t*)0XE000E180)
#define NVIC_ICER1			((__vo uint32_t*)0XE000E184)
#define NVIC_ICER2  		((__vo uint32_t*)0XE000E188)
#define NVIC_ICER3			((__vo uint32_t*)0XE000E18C)


/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR 	((__vo uint32_t*)0xE000E400)

/*
 * ARM Cortex Mx Processor number of priority bits implemented in Priority Register
 */
#define NO_PR_BITS_IMPLEMENTED  4

/* Base Addresses for Flash Memory and SRAM  */

#define FLASH_BASEADDR           0x08000000U
#define SRAM_BASEADDR            0x20000000U
#define ROM_BASEADDR             0x1FFF0000U


/*  AHBx and APBx Bus Peripheral Addresses  */

#define PERIPH_BASEADDR          0x40000000U
#define APB1_BASEADDR            PERIPH_BASEADDR
#define APB2_BASEADDR            0x40010000U
#define AHB1_BASEADDR            0x40020000U
#define AHB2_BASEADDR            0x50000000U


/*  AHB1 Bus Peripherals Base Addresses  */

#define GPIOA_BASEADDR           (AHB1_BASEADDR + 0x0000U)
#define GPIOB_BASEADDR           (AHB1_BASEADDR + 0x0400U)
#define GPIOC_BASEADDR           (AHB1_BASEADDR + 0x0800U)
#define GPIOD_BASEADDR           (AHB1_BASEADDR + 0x0C00U)
#define GPIOE_BASEADDR           (AHB1_BASEADDR + 0x1000U)
#define GPIOH_BASEADDR           (AHB1_BASEADDR + 0x1C00U)
#define CRC_BASEADDR             (AHB1_BASEADDR + 0x3000U)
#define RCC_BASEADDR             (AHB1_BASEADDR + 0x3800U)
#define FIR_BASEADDR             (AHB1_BASEADDR + 0x3C00U)
#define DMA1_BASEADDR            (AHB1_BASEADDR + 0x6000U)
#define DMA2_BASEADDR            (AHB1_BASEADDR + 0x6400U)


/*  APB1 Bus Peripherals Base Addresses  */

#define TIM2_BASEADDR            (APB1_BASEADDR + 0x0000U)
#define TIM3_BASEADDR            (APB1_BASEADDR + 0x0400U)
#define TIM4_BASEADDR            (APB1_BASEADDR + 0x0800U)
#define TIM5_BASEADDR            (APB1_BASEADDR + 0x0C00U)
#define RTC_BKP_BASEADDR         (APB1_BASEADDR + 0x2800U)
#define WWDG_BASEADDR            (APB1_BASEADDR + 0x2C00U)
#define IWDG_BASEADDR            (APB1_BASEADDR + 0x3000U)
#define I2S2EXT_BASEADDR         (APB1_BASEADDR + 0x3400U)
#define SPI2_BASEADDR            (APB1_BASEADDR + 0x3800U)
#define SPI3_BASEADDR            (APB1_BASEADDR + 0x3C00U)
#define I2S3EXT_BASEADDR         (APB1_BASEADDR + 0x4000U)
#define USART2_BASEADDR          (APB1_BASEADDR + 0x4400U)
#define I2C1_BASEADDR            (APB1_BASEADDR + 0x5400U)
#define I2C2_BASEADDR            (APB1_BASEADDR + 0x5800U)
#define I2C3_BASEADDR            (APB1_BASEADDR + 0x5C00U)
#define PWR_BASEADDR             (APB1_BASEADDR + 0x7000U)


/*  APB2 Bus Peripherals Base Addresses  */

#define TIM1_BASEADDR             (APB2_BASEADDR + 0x0000U)
#define TIM8_BASEADDR             (APB2_BASEADDR + 0x0400U)
#define USART1_BASEADDR           (APB2_BASEADDR + 0x1000U)
#define USART6_BASEADDR           (APB2_BASEADDR + 0x1400U)
#define ADC1_BASEADDR             (APB2_BASEADDR + 0x2000U)
#define SDIO_BASEADDR             (APB2_BASEADDR + 0x2C00U)
#define SPI1_BASEADDR             (APB2_BASEADDR + 0x3000U)
#define SPI4_BASEADDR             (APB2_BASEADDR + 0x3400U)
#define SYSCFG_BASEADDR           (APB2_BASEADDR + 0x3800U)
#define EXTI_BASEADDR             (APB2_BASEADDR + 0x3C00U)
#define TIM9_BASEADDR             (APB2_BASEADDR + 0x4000U)
#define TIM10_BASEADDR            (APB2_BASEADDR + 0x4400U)
#define TIM11_BASEADDR            (APB2_BASEADDR + 0x4800U)


/*  AHB2 Bus Peripherals Base Addresses  */

#define USB_OTG_BASEADDR         (APB2_BASEADDR + 0x0000U)


/*   Peripherals Definition (Peripherals Base Addresses TypeCasted to xxx_RegDef_t)  */

#define GPIOA                    ( (GPIO_RegDef_t*)GPIOA_BASEADDR )
#define GPIOB                    ( (GPIO_RegDef_t*)GPIOB_BASEADDR )
#define GPIOC                    ( (GPIO_RegDef_t*)GPIOC_BASEADDR )
#define GPIOD                    ( (GPIO_RegDef_t*)GPIOD_BASEADDR )
#define GPIOE                    ( (GPIO_RegDef_t*)GPIOE_BASEADDR )
#define GPIOH                    ( (GPIO_RegDef_t*)GPIOH_BASEADDR )

#define RCC                      ( (RCC_RegDef_t*)RCC_BASEADDR )
#define EXTI                     ( (EXTI_RegDef_t*)EXTI_BASEADDR )
#define SYSCFG                   ( (SYSCFG_RegDef_t*)SYSCFG_BASEADDR )
#define SPI1                     ( (SPI_RegDef_t*)SPI1_BASEADDR )
#define SPI2                     ( (SPI_RegDef_t*)SPI2_BASEADDR )
#define SPI3                     ( (SPI_RegDef_t*)SPI3_BASEADDR )
#define SPI4                     ( (SPI_RegDef_t*)SPI4_BASEADDR )

/*   GPIO Peripheral Structure  */
typedef struct {
	__vo	uint32_t MODER;             /* GPIO port mode register                                         Address Offset 0x00  */
	__vo	uint32_t OTYPER;            /* GPIO port output type register                                  Address Offset 0x04  */
	__vo	uint32_t OSPEEDR;           /* GPIO port output speed register                                 Address Offset 0x08  */
	__vo	uint32_t PUPDR;             /* GPIO port pull-up/pull-down register                            Address Offset 0x0C  */
	__vo	uint32_t IDR;               /* GPIO port input data register                                   Address Offset 0x10  */
	__vo	uint32_t ODR;               /* GPIO port output data register                                  Address Offset 0x14  */
	__vo	uint32_t BSRR;              /* GPIO port bit set/reset register                                Address Offset 0x18  */
	__vo	uint32_t LCKR;              /* GPIO port configuration lock register                           Address Offset 0x1C  */
	__vo	uint32_t AFR[2];            /* GPIO alternate function low register  AFR[0]                    Address Offset 0x20  */
                                        /* GPIO alternate function high register AFR[1]                    Address Offset 0x24  */
}GPIO_RegDef_t;

/*   RCC Peripheral Structure  */
typedef struct {
	__vo	uint32_t CR;                /* RCC clock control register                                      Address Offset 0x00  */
	__vo	uint32_t PLLCFGR;           /* RCC PLL configuration register                                  Address Offset 0x04  */
	__vo	uint32_t CFGR;              /* RCC clock configuration register                                Address Offset 0x08  */
	__vo	uint32_t CIR;               /* RCC clock interrupt register                                    Address Offset 0x0C  */
	__vo	uint32_t AHB1RSTR;          /* RCC AHB1 peripheral reset register                              Address Offset 0x10  */
	__vo	uint32_t AHB2RSTR;          /* RCC AHB2 peripheral reset register                              Address Offset 0x14  */
	    	uint32_t RESERVED0[2];      /* RESERVED 0x18 - 0x1C                                                                 */
	__vo	uint32_t APB1RSTR;          /* RCC APB1 peripheral reset register                              Address Offset 0x20  */
	__vo	uint32_t APB2RSTR;          /* RCC APB2 peripheral reset register                              Address Offset 0x24  */
	    	uint32_t RESERVED1[2];      /* RESERVED 0x28 - 0x2C                                                                 */
	__vo	uint32_t AHB1ENR;           /* RCC AHB1 peripheral clock enable register                       Address Offset 0x30  */
	__vo	uint32_t AHB2ENR;           /* RCC AHB2 peripheral clock enable register                       Address Offset 0x34  */
	    	uint32_t RESERVED2[2];      /* RESERVED 0x38 - 0x3C                                                                 */
	__vo	uint32_t APB1ENR;           /* RCC APB1 peripheral clock enable register                       Address Offset 0x40  */
	__vo	uint32_t APB2ENR;           /* RCC APB2 peripheral clock enable register                       Address Offset 0x44  */
	    	uint32_t RESERVED3[2];      /* RESERVED 0x48 - 0x4C                                                                 */
	__vo	uint32_t AHB1LPENR;         /* RCC AHB1 peripheral clock enable in low power mode register     Address Offset 0x50  */
	__vo	uint32_t AHB2LPENR;         /* RCC AHB2 peripheral clock enable in low power mode register     Address Offset 0x54  */
	    	uint32_t RESERVED4[2];      /* RESERVED 0x58 - 0x5C                                                                 */
	__vo	uint32_t APB1LPENR;         /* RCC APB1 peripheral clock enable in low power mode register     Address Offset 0x60  */
	__vo	uint32_t APB2LPENR;         /* RCC APB2 peripheral clock enabled in low power mode register    Address Offset 0x64  */
	    	uint32_t RESERVED5[2];      /* RESERVED 0x68 - 0x6C                                                                 */
	__vo	uint32_t BDCR;              /* RCC Backup domain control register                              Address Offset 0x70  */
	__vo	uint32_t CSR;               /* RCC clock control & status register                             Address Offset 0x74  */
	    	uint32_t RESERVED6[2];      /* RESERVED 0x78 - 0x7C                                                                 */
	__vo	uint32_t SSCGR;             /* RCC spread spectrum clock generation register                   Address Offset 0x80  */
	__vo	uint32_t PLLI2SCFGR;        /* RCC PLLI2S configuration register                               Address Offset 0x84  */
			uint32_t RESERVED7;         /* RESERVED 0x88                                                                        */
	__vo	uint32_t DCKCFGR;           /* RCC Dedicated Clocks Configuration Register                     Address Offset 0x8C  */
}RCC_RegDef_t;


/*   EXTI Peripheral Structure  */

typedef struct {
	__vo	uint32_t IMR;               /* Interrupt mask register                                         Address Offset 0x00  */
	__vo	uint32_t EMR;               /* Event mask register                                             Address Offset 0x04  */
	__vo	uint32_t RTSR;              /* Rising trigger selection register                               Address Offset 0x08  */
	__vo	uint32_t FTSR;              /* Falling trigger selection register                              Address Offset 0x0C  */
	__vo	uint32_t SWIER;             /* Software interrupt event register                               Address Offset 0x10  */
	__vo	uint32_t PR;                /* Pending register                                                Address Offset 0x14  */
}EXTI_RegDef_t;

/*   SYSCFG Peripheral Structure  */
typedef struct {
	__vo	uint32_t MEMRMP;            /* SYSCFG memory remap register                                    Address Offset 0x00  */
	__vo	uint32_t PMC;               /* SYSCFG peripheral mode configuration register                   Address Offset 0x04  */
	__vo	uint32_t EXTICR[4];         /* SYSCFG external interrupt configuration registers 1 - 4, Address Offset 0x08 - 0x14  */
	     	uint32_t RESERVED[2];       /* Reserved  0x18 - 0x1C                                                                */
	__vo	uint32_t CMPCR;             /* Compensation cell control register                              Address Offset 0x20  */
}SYSCFG_RegDef_t;

/*   SPI Peripheral Structure  */
typedef struct {
	__vo	uint32_t CR1;               /* SPI control register 1 (not used in I2S mode)                   Address Offset 0x00  */
	__vo    uint32_t CR2;               /* SPI control register 2                                          Address Offset 0x00  */
	__vo	uint32_t SR;                /* SPI control register 2                                          Address Offset 0x08  */
	__vo	uint32_t DR;                /* SPI data register                                               Address Offset 0x0C  */
	__vo   	uint32_t CRCPR;             /* SPI CRC polynomial register (not used in I2Smode)               Address Offset 0x10  */
	__vo	uint32_t RXCRCR;            /* SPI RX CRC register (not used in I2S mode)                      Address Offset 0x14  */
	__vo	uint32_t TXCRCR;            /* SPI TX CRC register (not used in I2S mode)                      Address Offset 0x18  */
	__vo	uint32_t I2SCFGR;           /* SPI_I2S configuration register                                  Address Offset 0x1C  */
	__vo	uint32_t I2SPR;             /* SPI_I2S prescaler register                                      Address Offset 0x20  */
}SPI_RegDef_t;

/*  Clock Enable Macros For GIPOx Peripherals   */

#define GPIOA_PCLK_EN()      ( RCC->AHB1ENR |= (1 << 0) )
#define GPIOB_PCLK_EN()      ( RCC->AHB1ENR |= (1 << 1) )
#define GPIOC_PCLK_EN()      ( RCC->AHB1ENR |= (1 << 2) )
#define GPIOD_PCLK_EN()      ( RCC->AHB1ENR |= (1 << 3) )
#define GPIOE_PCLK_EN()      ( RCC->AHB1ENR |= (1 << 4) )
#define GPIOH_PCLK_EN()      ( RCC->AHB1ENR |= (1 << 5) )


/*   Clock Enable Macros For I2C Peripherals    */

#define I2C1_PCLK_EN()       ( RCC->APB1ENR |= (1 << 21) )
#define I2C2_PCLK_EN()       ( RCC->APB1ENR |= (1 << 22) )
#define I2C3_PCLK_EN()       ( RCC->APB1ENR |= (1 << 23) )

/*  Clock Enable Macros For SPIx Peripherals    */

#define SPI1_PCLK_EN()       ( RCC->APB2ENR |= (1 << 12) )
#define SPI2_PCLK_EN()       ( RCC->APB1ENR |= (1 << 14) )
#define SPI3_PCLK_EN()       ( RCC->APB1ENR |= (1 << 15) )
#define SPI4_PCLK_EN()       ( RCC->APB2ENR |= (1 << 13) )


/*  Clock Enable Macros For USARTx Peripherals  */

#define USART1_PCLK_EN()     ( RCC->APB2ENR |= (1 << 4)  )
#define USART2_PCLK_EN()     ( RCC->APB1ENR |= (1 << 17) )
#define USART6_PCLK_EN()     ( RCC->APB2ENR |= (1 << 5)  )


/*  Clock Enable Macros For SYSCFG Peripheral   */

#define SYSCFG_PCLK_EN()     ( RCC->APB2ENR |= (1 << 14)  )

/*  Clock Disable Macros For GIPOx Peripherals   */

#define GPIOA_PCLK_DI()      ( RCC->AHB1ENR &= ~ (1 << 0) )
#define GPIOB_PCLK_DI()      ( RCC->AHB1ENR &= ~ (1 << 1) )
#define GPIOC_PCLK_DI()      ( RCC->AHB1ENR &= ~ (1 << 2) )
#define GPIOD_PCLK_DI()      ( RCC->AHB1ENR &= ~ (1 << 3) )
#define GPIOE_PCLK_DI()      ( RCC->AHB1ENR &= ~ (1 << 4) )
#define GPIOH_PCLK_DI()      ( RCC->AHB1ENR &= ~ (1 << 5) )

/*   Clock Disable Macros For I2C Peripherals    */

#define I2C1_PCLK_DI()       ( RCC->APB1ENR &= ~ (1 << 21) )
#define I2C2_PCLK_DI()       ( RCC->APB1ENR &= ~ (1 << 22) )
#define I2C3_PCLK_DI()       ( RCC->APB1ENR &= ~ (1 << 23) )


/*  Clock Disable Macros For SPIx Peripherals    */

#define SPI1_PCLK_DI()       ( RCC->APB2ENR &= ~ (1 << 12) )
#define SPI2_PCLK_DI()       ( RCC->APB1ENR &= ~ (1 << 21) )
#define SPI3_PCLK_DI()       ( RCC->APB1ENR &= ~ (1 << 22) )
#define SPI4_PCLK_DI()       ( RCC->APB2ENR &= ~ (1 << 13) )


/*  Clock Disable Macros For USARTx Peripherals  */

#define USART1_PCLK_DI()     ( RCC->APB2ENR &= ~ (1 << 4)  )
#define USART2_PCLK_DI()     ( RCC->APB1ENR &= ~ (1 << 17) )
#define USART6_PCLK_DI()     ( RCC->APB2ENR &= ~ (1 << 5)  )

/*
 * Macros to Reset GPIOx Peripherals
 */

#define GPIOA_REG_RESET()   do{ ( RCC->AHB1RSTR |= (1 << 0)); ( RCC->AHB1RSTR &= ~ (1 << 0)); }while(0) // Set the bit Then Reset it
#define GPIOB_REG_RESET()   do{ ( RCC->AHB1RSTR |= (1 << 1)); ( RCC->AHB1RSTR &= ~ (1 << 1)); }while(0)
#define GPIOC_REG_RESET()   do{ ( RCC->AHB1RSTR |= (1 << 2)); ( RCC->AHB1RSTR &= ~ (1 << 2)); }while(0)
#define GPIOD_REG_RESET()   do{ ( RCC->AHB1RSTR |= (1 << 3)); ( RCC->AHB1RSTR &= ~ (1 << 3)); }while(0)
#define GPIOE_REG_RESET()   do{ ( RCC->AHB1RSTR |= (1 << 4)); ( RCC->AHB1RSTR &= ~ (1 << 4)); }while(0)
#define GPIOH_REG_RESET()   do{ ( RCC->AHB1RSTR |= (1 << 5)); ( RCC->AHB1RSTR &= ~ (1 << 7)); }while(0)

/*
 * Macros to Reset SPIx Peripherals
 */
#define SPI1_REG_RESET()   do{ ( RCC->APB2RSTR |= (1 << 12)); ( RCC->APB2RSTR &= ~ (1 << 12)); }while(0) // Set the bit Then Reset it
#define SPI2_REG_RESET()   do{ ( RCC->APB1RSTR |= (1 << 14)); ( RCC->APB1RSTR &= ~ (1 << 14)); }while(0)
#define SPI3_REG_RESET()   do{ ( RCC->APB1RSTR |= (1 << 15)); ( RCC->APB1RSTR &= ~ (1 << 15)); }while(0)
#define SPI4_REG_RESET()   do{ ( RCC->APB2RSTR |= (1 << 13)); ( RCC->APB2RSTR &= ~ (1 << 13)); }while(0)

/*
 * Macro to return a code between 0 - 7 for a given GPIO base address (x)
 * Used to select the port in SYSCFG external interrupt configuration registers EXTICRx
 */
#define GPIO_BASEADDR_TO_CODE(x) ( (x == GPIOA)?0:\
                                   (x == GPIOB)?1:\
                                   (x == GPIOC)?2:\
                                   (x == GPIOD)?3:\
                                   (x == GPIOE)?4:\
                                   (x == GPIOH)?7:0 )

/*
 * IRQ(Interrupt Request) Numbers FOR stm32f401xx MCU
 */

#define IRQ_NO_EXTI0 		6
#define IRQ_NO_EXTI1 		7
#define IRQ_NO_EXTI2 		8
#define IRQ_NO_EXTI3 		9
#define IRQ_NO_EXTI4 		10
#define IRQ_NO_EXTI9_5 		23
#define IRQ_NO_EXTI15_10 	40
#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2         36
#define IRQ_NO_SPI3         51
#define IRQ_NO_SPI4
#define IRQ_NO_I2C1_EV     31
#define IRQ_NO_I2C1_ER     32
#define IRQ_NO_USART1	    37
#define IRQ_NO_USART2	    38
#define IRQ_NO_USART3	    39
#define IRQ_NO_UART4	    52
#define IRQ_NO_UART5	    53
#define IRQ_NO_USART6	    71

/******************************************************************************************
 *                         Some Generic Macros
 ******************************************************************************************/
#define ENABLE              1
#define DISABLE             0
#define SET                 ENABLE
#define RESET               DISABLE
#define GPIO_PIN_SET        SET
#define GPIO_PIN_RESET      RESET
#define FLAG_SET            SET
#define FLAG_RESET          RESET

/******************************************************************************************
 *             Bit position definitions of SPI peripheral
 ******************************************************************************************/
/*
 * Bit position definitions SPI_CR1
 */
#define SPI_CR1_CPHA     				 0
#define SPI_CR1_CPOL      				 1
#define SPI_CR1_MSTR     				 2
#define SPI_CR1_BR   					 3
#define SPI_CR1_SPE     				 6
#define SPI_CR1_LSBFIRST   			 	 7
#define SPI_CR1_SSI     				 8
#define SPI_CR1_SSM      				 9
#define SPI_CR1_RXONLY      		 	10
#define SPI_CR1_DFF     			 	11
#define SPI_CR1_CRCNEXT   			 	12
#define SPI_CR1_CRCEN   			 	13
#define SPI_CR1_BIDIOE     			 	14
#define SPI_CR1_BIDIMODE      			15

/*
 * Bit position definitions SPI_CR2
 */
#define SPI_CR2_RXDMAEN		 			0
#define SPI_CR2_TXDMAEN				 	1
#define SPI_CR2_SSOE				 	2
#define SPI_CR2_FRF						4
#define SPI_CR2_ERRIE					5
#define SPI_CR2_RXNEIE				 	6
#define SPI_CR2_TXEIE					7


/*
 * Bit position definitions SPI_SR
 */
#define SPI_SR_RXNE						0
#define SPI_SR_TXE				 		1
#define SPI_SR_CHSIDE				 	2
#define SPI_SR_UDR					 	3
#define SPI_SR_CRCERR				 	4
#define SPI_SR_MODF					 	5
#define SPI_SR_OVR					 	6
#define SPI_SR_BSY					 	7
#define SPI_SR_FRE					 	8



#endif /* INC_STM32F401XX_H_ */
