/*
 * stm32f401xx_gpio_driver.c
 *
 *  Created on: 14 Jun 2021
 *      Author: hany.elmadany
 */

#include "stm32f401xx_gpio_driver.h"


/*
 * Peripheral Clock Setup
 */
/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
	}
	else
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
	}

}

/*
 * Init and De-Init
 */

/*********************************************************************
 * @fn      		  - GPIO_Init
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
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);// Enable the GPIOx Port
	uint32_t temp = 0;
	if (pGPIOHandle->GPIO_pinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		// configure the NoNe Interrupt mode of gpio pin
		temp = ( pGPIOHandle->GPIO_pinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber) );
		pGPIOHandle->pGPIOx->MODER &= ~ (0x3 << (2 * pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber) );      // Clearing required bits
		pGPIOHandle->pGPIOx->MODER |= temp;
	}
	else
	{
		// configure the Interrupt mode of gpio pin
		if(pGPIOHandle->GPIO_pinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//configure FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber);

			//clear the corresponding RTSR bit
			EXTI->RTSR &= ~ (1 << pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber);

		}else if(pGPIOHandle->GPIO_pinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//configure RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber);

			//clear the corresponding FTSR bit
			EXTI->FTSR &= ~ (1 << pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber);
		}else if(pGPIOHandle->GPIO_pinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			//configure both FTSR and RTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber);
		}
		// configure the GPIO port Selection in SYSCFG_EXTICR
		uint8_t temp1, temp2, port_code;
		temp1 = pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber / 4; // EXTICR Register Select from EXTICR1 to EXTICR4
		temp2 = pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber % 4; // EXTI line Number
		port_code = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx); // Macro Function to obtain the Code of the Selected PORT
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] |= ( port_code << (4 * temp2) );

		// Enable the exti interrupt delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber);
	}
	temp = 0;

	// configure the speed of gpio output pin
	temp = ( pGPIOHandle->GPIO_pinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->OSPEEDR &= ~ (0x3 << (2 * pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber));      // Clearing required bits
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;

	// configure the Pull up / Pull down of gpio  pin
	temp = ( pGPIOHandle->GPIO_pinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->PUPDR &= ~ (0x3 << (2 * pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber));      // Clearing required bits
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;

	// configure the Output Type of gpio  pin
	temp = ( pGPIOHandle->GPIO_pinConfig.GPIO_OPType << (pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->OTYPER &= ~ (0x1 << pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber);      // Clearing required bits
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;

	// configure the alternative functionality of gpio pin
	if (pGPIOHandle->GPIO_pinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint8_t temp1, temp2;
		temp1 = pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~ ( 0xF << (4 * temp2) );      // Clearing required bits
		pGPIOHandle->pGPIOx->AFR[temp1] |= ( pGPIOHandle->GPIO_pinConfig.GPIO_PinAltFunMode << (4 * temp2) );
	}


}
/*********************************************************************
 * @fn      		  - GPIO_DeInit
 *
 * @brief             -
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if (pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}else if (pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}else if (pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}else if (pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}else if (pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}else if (pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
}

/*
 * Data read and write
 */
/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPin
 *
 * @brief             -
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - Pin Number Which required to read from
 * @param[in]         -
 *
 * @return            -  1 or 0
 *
 * @Note              -  none
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t) ((pGPIOx->IDR >> PinNumber) & 0x01);
	return value;
}
/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPort
 *
 * @brief             -
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  return from the port up to 16 bit (16 Pins per Port)
 *
 * @Note              -  none
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
		uint16_t value;
		value = (uint16_t) pGPIOx->IDR;
		return value;
	return 0;
}
/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPin
 *
 * @brief             -
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - Pin Number Which required to Write to
 * @param[in]         - Value to write to the pin selected
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value)
{
	if (value == GPIO_PIN_SET)
	{
		// Write 1 to the output data register at the bit field corresponding to the pin number
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else
	{
		// Write 0
		pGPIOx->ODR &= ~ (1 << PinNumber);
	}
}
/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPort
 *
 * @brief             -
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - Value to write to the pin selected
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value)
{
		pGPIOx->ODR = value;
}
/*********************************************************************
 * @fn      		  - GPIO_ToggleOutputPin
 *
 * @brief             -
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - Pin Number Which required to toggle
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}

/*
 * IRQ Configuration and ISR Handling
 */
/*********************************************************************
 * @fn      		  - GPIO_IRQInterruptConfig
 *
 * @brief             -
 *
 * @param[in]         - IRQ Number from the vector table
 * @param[in]         - Enable or Disable The interrupt
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(IRQNumber <= 31)
	{
		//ISER0 Register
		*NVIC_ISER0 |= ( 1 << (IRQNumber) );

	}else if (IRQNumber > 31 && IRQNumber < 64) // 32 to 63
	{
		//ISER1 Register
		*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );

	}else if (IRQNumber >= 64 && IRQNumber < 96) // 64 to 95
	{
		//ISER2 Register
		*NVIC_ISER2 |= ( 1 << (IRQNumber % 64) );
	}
}
/*********************************************************************
 * @fn      		  - GPIO_IRQPriorityConfig
 *
 * @brief             -
 *
 * @param[in]         -IRQ Number from the vector table
 * @param[in]         -IRQ Priority Required
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	//Find out the IPR Register Number
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (iprx_section * 8) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}
/*********************************************************************
 * @fn      		  - GPIO_IRQHandling
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
	//Clear the EXTI PR corresponding to the Pin Number
	if(EXTI->PR & (1<<PinNumber))
	{
		//Clear
		EXTI->PR |= (1<<PinNumber);
	}
}


