/** ***************************************************************************
 * @file handle_packet.c functions for handling serial UCB packets and CRM
 *       packets
 * @brief some of the callbacks for message handling of non-sensor data to be
 *        sent to Nav-View eg "PING".
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

#include <stdint.h>
#include <GlobalConstants.h>
#include "sensors_data.h"
#include "sensorsAPI.h"

sensors_data_t gSensorsData;



void GetAccelData_g(double *data)
{
    for(int i = 0; i < 3; i++){
        data[i] = gSensorsData.scaledSensors[i+XACCEL];
    }
}

void GetAccelData_mPerSecSq(double *data)
{
    for(int i = 0; i < 3; i++){
        data[i] = gSensorsData.scaledSensors[i+XACCEL] * g_TO_M_SEC_SQ;
    }
}

void GetRateData_radPerSec(double *data)
{
    for(int i = 0; i < 3; i++){
        data[i] = gSensorsData.scaledSensors[i+XRATE];
    }
}

void GetRateData_degPerSec(double *data)
{
    for(int i = 0; i < 3; i++){
        data[i] = gSensorsData.scaledSensors[i+XRATE] * RAD_TO_DEG;
    }
}

void GetMagData_G(double *data)
{
    for(int i = 0; i < 3; i++){
        data[i] = gSensorsData.scaledSensors[i+XMAG];
    }
}

void GetAccelsTempData(double *data)
{
    for(int i = 0; i < 3; i++){
        data[i] = gSensorsData.scaledSensors[i+9];
    }
}

void GetRatesTempData(double *data)
{
    for(int i = 0; i < 3; i++){
        data[i] = gSensorsData.scaledSensors[i+12];
    }
}

void GetBoardTempData(double *data)
{
    *data = gSensorsData.scaledSensors[15];
}

