/*
 * stm32f401xx_spi_driver.c
 *
 *  Created on: 18 Jun 2021
 *      Author: hany.elmadany
 */

#include"stm32f401xx_spi_driver.h"

/*
 * Peripheral Clock Setup
 */
/*********************************************************************
 * @fn      		  - SPI_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given SPIx
 *
 * @param[in]         - base address of the SPIx peripheral
 * @param[in]         - ENABLE or DISABLE Macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}else if (pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}else if (pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}else if (pSPIx == SPI4)
		{
			SPI4_PCLK_EN();
		}
	}
	else
	{
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}else if (pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}else if (pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}else if (pSPIx == SPI4)
		{
			SPI4_PCLK_EN();
		}
	}
}

/*
 * Init and De-Init
 */
/*********************************************************************
 * @fn      		  - SPI_Init
 *
 * @brief             -
 *
 * @param[in]         - Pointer to the Handle Structure Given by the user
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//Enable Peripheral Clock
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);
	// Configure SPI_CR1 Register
	uint32_t tempReg = 0;
	// Configure the Device Mode
	tempReg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;
	// Configure The Bus Config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//BIDIMODE = 0
		tempReg &= ~ (1<<SPI_CR1_BIDIMODE);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//BIDIMODE = 1
		tempReg |= (1<<SPI_CR1_BIDIMODE);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//BIDIMODE = 0
		tempReg &= ~ (1<<SPI_CR1_BIDIMODE);
		//RxONLY = 1
		tempReg |= (1<<SPI_CR1_RXONLY);
	}
	// Configure Sclk Speed
	tempReg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;
	// Configure Data Frame Format DFF
	tempReg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;
	// Configure Clk Polarity
	tempReg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;
	// Configure Clk Phase
	tempReg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;
	// Configure Software Slave Managment
	tempReg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;
	pSPIHandle->pSPIx->CR1 = tempReg;
}
/*********************************************************************
 * @fn      		  - SPI_DeInit
 *
 * @brief             -
 *
 * @param[in]         - base address of the SPIx peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if (pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}else if (pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}else if (pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}else if (pSPIx == SPI4)
	{
		SPI4_REG_RESET();
	}
}

/*
 * Get Flags
 */
/*********************************************************************
 * @fn      		  - SPI_GetFlagStatus
 *
 * @brief             -
 *
 * @param[in]         - base address of the SPIx peripheral
 * @param[in]         - Flage Name (Flag bit position Macro)
 * @param[in]         -
 *
 * @return            -  Status of required flag
 *
 * @Note              -  none
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t * pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*
 * Data Send and Receive
 */
/*********************************************************************
 * @fn      		  - SPI_SendData
 *
 * @brief             -
 *
 * @param[in]         - base address of the SPIx peripheral
 * @param[in]         - Transmit Buffer
 * @param[in]         - Length
 *
 * @return            -  none
 *
 * @Note              -  Blocking Call Function
 */
void SPI_SendData (SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len)
{
	while (len > 0)
	{	// Wait untill TXE Flag is SET
		while ( SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET );
		if ( pSPIx->CR1 & (1<<SPI_CR1_DFF) )
		{
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			len--;
			len--;
			(uint16_t*)pTxBuffer++;
		}
		else
		{
			pSPIx->DR = *pTxBuffer;
			len--;
			pTxBuffer++;
		}

	}
}
/*********************************************************************
 * @fn      		  - SPI_ReceiveData
 *
 * @brief             -
 *
 * @param[in]         - base address of the SPIx peripheral
 * @param[in]         - Receive Buffer
 * @param[in]         - Length
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void SPI_ReceiveData (SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len)
{
	while (len > 0)
	{	// Wait untill RXNE Flag is SET
		while ( SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET );
		if ( pSPIx->CR1 & (1<<SPI_CR1_DFF) )
		{
			*((uint16_t*)pRxBuffer) = pSPIx->DR ;
			len--;
			len--;
			(uint16_t*)pRxBuffer++;
		}
		else
		{
			*pRxBuffer = pSPIx->DR;
			len--;
			pRxBuffer++;
		}

	}
}

/*
 * IRQ Configuration and ISR Handling
 */
/*********************************************************************
 * @fn      		  - SPI_IRQInterruptConfig
 *
 * @brief             -
 *
 * @param[in]         - IRQ Number from the vector table
 * @param[in]         - ENABLE or DISABLE
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

}
/*********************************************************************
 * @fn      		  - SPI_IRQPriorityConfig
 *
 * @brief             -
 *
 * @param[in]         - IRQ Number from the vector table
 * @param[in]         - IRQ Priority Required
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{

}
/*********************************************************************
 * @fn      		  - SPI_IRQHandling
 *
 * @brief             -
 *
 * @param[in]         - Pointer to the Handle Structure Given by the user
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void SPI_IRQHandling(SPI_Handle_t *pHandle)
{

}
/*
 * Other Peripheral Control APIs
 */
/*********************************************************************
 * @fn      		  - SPI_PeripheralControl
 *
 * @brief             -
 *
 * @param[in]         -base address of the SPIx peripheral
 * @param[in]         -ENABLE or DISABLE Macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1<<SPI_CR1_SPE);
	}
	else
	{
		pSPIx->CR1 &= ~ (1<<SPI_CR1_SPE);
	}
}
/*********************************************************************
 * @fn      		  - SPI_SSIConfig
 *
 * @brief             -
 *
 * @param[in]         -base address of the SPIx peripheral
 * @param[in]         -ENABLE or DISABLE Macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  It should be Config in Case of SSM enable or Multi Master Mode
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1<<SPI_CR1_SSI);
	}
	else
	{
		pSPIx->CR1 &= ~ (1<<SPI_CR1_SSI);
	}
}
/*********************************************************************
 * @fn      		  - SPI_SSOEConfig
 *
 * @brief             -
 *
 * @param[in]         -base address of the SPIx peripheral
 * @param[in]         -ENABLE or DISABLE Macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  It should be Config in Case of SSM Disabled and Multi tasking mode
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR2 |= (1<<SPI_CR2_SSOE);
	}
	else
	{
		pSPIx->CR2 &= ~ (1<<SPI_CR2_SSOE);
	}
}



