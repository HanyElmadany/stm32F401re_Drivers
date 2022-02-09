/*
 * stm32f401xx_spi_driver.h
 *
 *  Created on: 18 Jun 2021
 *      Author: hany.elmadany
 */

#ifndef INC_STM32F401XX_SPI_DRIVER_H_
#define INC_STM32F401XX_SPI_DRIVER_H_

#include"stm32f401xx.h"

/*    Configuration Structure for a SPIx     */
typedef struct {
	uint8_t SPI_DeviceMode;                       /*!< possible values from @SPI_DeviceMode >*/
	uint8_t SPI_BusConfig;                        /*!< possible values from @SPI_BusConfig > */
	uint8_t SPI_SclkSpeed;                        /*!< possible values from @SPI_SclkSpeed > */
	uint8_t SPI_DFF;                              /*!< possible values from @SPI_DFF >       */
	uint8_t SPI_CPOL;                             /*!< possible values from @SPI_CPOL >      */
	uint8_t SPI_CPHA;                             /*!< possible values from @SPI_CPHA >      */
	uint8_t SPI_SSM;                              /*!< possible values from @SPI_SSM >       */
}SPI_Config_t;


/*        Handle Structure for a SPIx       */
typedef struct {
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPIConfig;
}SPI_Handle_t;


/*
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MOD_MASTER                   1       /* Device as Mode Master                                               */
#define SPI_DEVICE_MOD_SLAVE                    0       /* Device as Mode Slave                                                */

/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD                       1       /* Bus Configuration Full Duplex                                       */
#define SPI_BUS_CONFIG_HD                       2       /* Bus Configuration Half Duplex                                       */
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY           3       /* Bus Configuration Simplex Rx Only                                   */
                                                        /* For Simplex Tx Only ->Config full duplex and remove the Rx Line     */

/*
 * @SPI_SclkSpeed
 * Baud rate control options from Fpclk/2 to Fpclk/256
 */
#define SPI_SCLK_DIV2                           0
#define SPI_SCLK_DIV4                           1
#define SPI_SCLK_DIV8                           2
#define SPI_SCLK_DIV16                          3
#define SPI_SCLK_DIV32                          4
#define SPI_SCLK_DIV64                          5
#define SPI_SCLK_DIV128                         6
#define SPI_SCLK_DIV256                         7

/*
 * @SPI_DFF
 * Data Frame Format Selection
 */
#define SPI_DFF_8BITS                           0
#define SPI_DFF_16BITS                          1

/*
 * @SPI_CPOL
 * Sclk Polarity Selection
 */
#define SPI_CPOL_HIGH                           1
#define SPI_CPOL_LOW                            0

/*
 * @SPI_CPHA
 * Sclk Phase Selection
 */
#define SPI_CPHA_HIGH                           1
#define SPI_CPHA_LOW                            0

/*
 * @SPI_SSM
 * Software Slave Management
 */
#define SPI_SSM_EN                              1
#define SPI_SSM_DI                              0

/*
 * SPI Related Status Flags Definitions
 */
#define SPI_TXE_FLAG                            ( 1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG                           ( 1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG                           ( 1 << SPI_SR_BSY)




/*************************************************************************************************
 *                           APIs Supported by this Driver
 *           For more information about the APIs check the function definitions
 *************************************************************************************************/

/*
 * Peripheral Clock Setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*
 * Init and De-Init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Get Flags
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t * pSPIx, uint32_t FlagName);

/*
 * Data Send and Receive
 */
void SPI_SendData (SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len);
void SPI_ReceiveData (SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len);

/*
 * IRQ Configuration and ISR Handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

/*
 * Other Peripheral Control APIs
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);







#endif /* INC_STM32F401XX_SPI_DRIVER_H_ */
