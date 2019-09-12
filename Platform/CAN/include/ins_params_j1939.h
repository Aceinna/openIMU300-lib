/** ***************************************************************************
 * @file sae_j1939.h definitions of SAE J1939 standard
 * @Author Feng
 * @date   Feb. 2017
 * @brief  Copyright (c) 2017 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *****************************************************************************/
#ifndef INS_PARAMS_J1939_H
#define INS_PARAMS_J1939_H
#include <stdint.h>

typedef struct {
	uint8_t status;
	double 	latitude;							// .0000001 deg/bit
	double 	longitude;							// 0.0000001 deg/bit
    float   compassBearing;    					// 0.0078125  deg/bit
    float   navSpeed;          					// 0.00390625 kph/bit
    float   pitch;             					// 0.0078125  deg/bit
    float   altitude;          					// 0.125      m/bit
    float   frontAxleSpeed;                     // 0.00390625 kph/bit
    float   relSpeedFrontAxleLeftWheel;         // 0.0625 km/h per bit
    float   relSpeedFrontAxleRightWheel;        // 0.0625 km/h per bit 
    float   relSpeedRearAxle1LeftWheel;         // 0.0625 km/h per bit
    float   relSpeedRearAxle1RigthWheel;        // 0.0625 km/h per bit
    float   relSpeedRearAxle2LeftWheel;         // 0.0625 km/h per bit
    float   relSpeedRearAxle2RigthWheel;        // 0.0625 km/h per bit 
}j1939_ins_params_t;

extern j1939_ins_params_t canInsParams;

#define J1939_INS_POSITION_DATA_UPDATED    0x01
#define J1939_INS_ALTITUDE_DATA_UPDATED    0x02
#define J1939_INS_SPEED_DATA_UPDATED       0x04


#endif // SAE_J1939_H