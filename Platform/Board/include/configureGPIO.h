/** ***************************************************************************
 * @file configureGPIO.h BSP call to set up GPIO pins
 * @Author
 * @date   September, 2008
 * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *****************************************************************************/
#ifndef _CONFIGURE_IO_H_
#define _CONFIGURE_IO_H_

#include <stdint.h>

#include "stm32f4xx.h"
#include "boardDefinition.h"

#define GPIO_INPUT  0
#define GPIO_OUTPUT 1

// Define local functions
void InitCommonPins_GPIO( void );

void InitPin_GPIO( uint32_t      PeriphClock,
                   GPIO_TypeDef* GPIO_Port,
                   uint32_t      GPIO_Pin,
                   uint8_t       inputOutputSelector );

void InitPin_GPIO_PDN( uint32_t      PeriphClock,
                   GPIO_TypeDef* GPIO_Port,
                   uint32_t      GPIO_Pin,
                   uint8_t       inputOutputSelector );

//  from main() to set up the GPIO pins and determine the
//   board configuration based on three GPIOs.
void InitBoardConfiguration_GPIO(void);
uint8_t ReadUnitConfiguration_GPIO(void);
uint8_t GPIO_DetectTestMode();
void    GPIO_InitPinsForTestMode();
void    ControlPortInit(void);

#endif