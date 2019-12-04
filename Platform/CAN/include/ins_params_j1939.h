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

// NAV data updated from CAN bit flags 
#define J1939_GPS_POSITION_DATA_UPDATED    0x01
#define J1939_GPS_ALTITUDE_DATA_UPDATED    0x02
#define J1939_VEH_SPEED_DATA_UPDATED       0x04
#define J1939_GNSS_DOP_DATA_UPDATED        0x08

typedef struct {
	uint8_t status;
	double 	latitude;							// deg +
	double 	longitude;							// deg +
    float   gpsBearing;    					    // deg +
    float   navSpeed;          					// kph +
    float   pitch;             					// deg  ----------------
    float   altitude;          					// m   +
    int     numSats;                            // count
    float   hDOP;                               // ratio
    float   vDOP;                               // ratio
    float   pDOP;                               // ratio
    float   tDOP;                               // ratio
//****************************************************** 
    float   frontAxleSpeed;                     // kph
    float   relSpeedFrontAxleLeftWheel;         // km/h
    float   relSpeedFrontAxleRightWheel;        // km/h
    float   relSpeedRearAxle1LeftWheel;         // km/h
    float   relSpeedRearAxle1RigthWheel;        // km/h
    float   relSpeedRearAxle2LeftWheel;         // km/h
    float   relSpeedRearAxle2RigthWheel;        // km/h
}j1939_input_nav_params_t;

extern j1939_input_nav_params_t canInputNavParams;


#endif // SAE_J1939_H