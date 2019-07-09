/** ***************************************************************************
 * @file watchdog.h cpu hardware watchdog timer interface functions
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *****************************************************************************/
/*******************************************************************************
Copyright 2018 ACEINNA, INC

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*******************************************************************************/
#ifndef _WATCHDOG_H_
#define _WATCHDOG_H_

#include "stm32f4xx_iwdg.h"

#define   PetWatchdog();   IWDG->KR = KR_KEY_RELOAD; // 0xaaaa

#define  ENABLE_WATCHDOG  0

// @brief <pre> Specify the IWDG Reload value
//   Eqn:
//        delay [ msec ] = 0.125 [ msec ] ( PSC/4 ) * ( IWDG->RLR + 1 )
//<br>
//    IWDG->RLR = ( delay [msec]/( 0.125 [msec] * ( PSC/4 ) ) ) - 1
// Prescaler (0xfff = 4095)
// Divider  PR bits Min timeout [ms] Max timeout [ms]
//     /4 = 0x00;   0.125            512                                    <br>
//     /8 = 0x01;   0.25            1024                                    <br>
//    /16 = 0x02;   0.5             2048                                    <br>
//    /32 = 0x03;   1.0             4096                                    <br>
//    /64 = 0x04;   2.0             8192                                    <br>
//   /128 = 0x05;   4.0            16384                                    <br>
//   /256 = 0x06;   8.0            32768                                    <br>
// <\pre>
#define NOMINAL_WATCHDOG_PRESCALER  IWDG_Prescaler_256 // 0x06
#define NOMINAL_WATCHDOG_DELAY      0x05 // delay corresponds to 10 DA task frames
                                         //   (with some headroom)

void InitTimer_Watchdog( FunctionalState NewState );
void SetMaxDelay_Watchdog( void );
void RestoreDelay_Watchdog( void );

#endif