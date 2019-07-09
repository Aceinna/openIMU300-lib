/** ***************************************************************************
 * @file   SPIWriteCallbackTable.h
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 * @details
 * Set up client SPI3 DMA and register handler
 * DKH added mag and spi 10.17.14
 ******************************************************************************/
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

#ifndef _SPI_WRITE_CALLBACK_TABLE_H_
#define _SPI_WRITE_CALLBACK_TABLE_H_

// These aren't advertised to the user at this point
void Accel_Output_Config_Write(uint8_t data); // 0x70
void Rate_Output_Config_Write(uint8_t data);  // 0x71
void Temp_Output_Config_Write(uint8_t data);  // 0x72
void Mag_Output_Config_Write(uint8_t data);   // 0x73

#endif