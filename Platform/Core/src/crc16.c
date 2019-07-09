
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

#include "stdint.h"
#include "crc16.h"


uint16_t CalculateCRC (uint8_t *buf, uint16_t  length)
{
	uint16_t crc = 0x1D0F;  //non-augmented inital value equivalent to the augmented initial value 0xFFFF
	
	for (int i=0; i < length; i++) {
		crc ^= buf[i] << 8;
		
		for (int j=0; j<8; j++) {
			if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            }
			else {
                crc = crc << 1;
            }
		}
	}
	
	return ((crc << 8 ) & 0xFF00) | ((crc >> 8) & 0xFF);
}
