/*
 * stm32f401xx_gpio_driver.h
 *
 *  Created on: 14 Jun 2021
 *      Author: hany.elmadany
 */

#ifndef INC_STM32F401XX_GPIO_DRIVER_H_
#define INC_STM32F401XX_GPIO_DRIVER_H_

#include "stm32f401xx.h"


/*    Configuration Structure for a GPIO pin     */
typedef struct {
	uint8_t GPIO_PinNumber;               /*!< possible values from @GPIO_PIN_NUMBERS >     */
	uint8_t GPIO_PinMode;                 /*!< possible values from @GPIO_PIN_MODES >       */
	uint8_t GPIO_PinSpeed;                /*!< possible values from @GPIO_PIN_SPEEDS >      */
	uint8_t GPIO_PinPuPdControl;          /*!< possible values from @GPIO_PU_PD_CONFIG >    */
	uint8_t GPIO_OPType;                  /*!< possible values from @GPIO_PIN_OUTPUT_TYPE > */
	uint8_t GPIO_PinAltFunMode;           /*!< possible values from @Reference Manual >     */
}GPIO_PinConfig_t;


/*        Handle Structure for a GPIO pin       */
typedef struct {
	//Pointer to Hold the base address of the GPIO peripheral
	GPIO_RegDef_t *pGPIOx;     /*  this pointer holds the base address of the GPIO port to which the pin belongs */
	GPIO_PinConfig_t GPIO_pinConfig;
}GPIO_Handle_t;

/*
 * @GPIO_PIN_NUMBERS
 */
#define GPIO_PIN_NO_0         0
#define GPIO_PIN_NO_1         1
#define GPIO_PIN_NO_2         2
#define GPIO_PIN_NO_3         3
#define GPIO_PIN_NO_4         4
#define GPIO_PIN_NO_5         5
#define GPIO_PIN_NO_6         6
#define GPIO_PIN_NO_7         7
#define GPIO_PIN_NO_8         8
#define GPIO_PIN_NO_9         9
#define GPIO_PIN_NO_10        10
#define GPIO_PIN_NO_11        11
#define GPIO_PIN_NO_12        12
#define GPIO_PIN_NO_13        13
#define GPIO_PIN_NO_14        14
#define GPIO_PIN_NO_15        15

/*
 * @GPIO_PIN_MODES
 */
#define GPIO_MODE_IN          0             /*  Input (reset state)                         00        */
#define GPIO_MODE_OUT         1             /*  General purpose output mode                 01        */
#define GPIO_MODE_ALTFN       2             /*  Alternate function mode                     10        */
#define GPIO_MODE_ANALOG      3             /*  Analog mode                                 11        */
#define GPIO_MODE_IT_FT       4             /*  Interrupt On Failing Edge             (User Defined)  */
#define GPIO_MODE_IT_RT       5             /*  Interrupt On Raising Edge             (User Defined)  */
#define GPIO_MODE_IT_RFT      6             /*  Interrupt On Raising Failing Edge     (User Defined)  */

/*
 * @GPIO_PIN_OUTPUT_TYPE
 */

#define GPIO_OP_TYPE_PP       0             /*  Output push-pull (reset state)              0         */
#define GPIO_OP_TYPE_OD       1             /*  Output open-drain                           1         */

/*
 * @GPIO_PIN_SPEEDS
 */

#define GPIO_SPEED_LOW        0             /*  Low speed                                   00        */
#define GPIO_SPEED_MEDIUM     1             /*  Medium speed                                01        */
#define GPIO_SPEED_FAST       2             /*  High Speed                                  10        */
#define GPIO_SPEED_HIGH       3             /*  Very High Speed                             11        */

/*
 * @GPIO_PU_PD_CONFIG
 * GPIO Pin Pull up and Pull Down Configuration Macros
 */

#define GPIO_NO_PUPD           0             /*  No pull-up, pull-down                       00        */
#define GPIO_PIN_PU            1             /*  Pull-up                                     01        */
#define GPIO_PIN_PD            2             /*  Pull-down                                   10        */
                                             /*  Reserved                                    11        */

/*************************************************************************************************
 *                           APIs Supported by this Driver
 *           For more information about the APIs check the function definitions
 *************************************************************************************************/


/*
 * Peripheral Clock Setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * Init and De-Init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ Configuration and ISR Handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);






#endif /* INC_STM32F401XX_GPIO_DRIVER_H_ */
