/** ***************************************************************************
 * @file sensorsAPI.h API functions for Magnitometer functionality
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

#ifndef _SENSORS_API_H
#define _SENSORS_API_H

#include <stdint.h>

/** ****************************************************************************
 * @name GetAccelData_g
 * @brief Get scaled accelerometer data in G
 * @param [in] data - pointer to external data structure
 * @retval N/A
 ******************************************************************************/
void  GetAccelData_g(double *data);

/** ****************************************************************************
 * @name GetAccelData_mPerSecSq
 * @brief Get scaled accelerometer data in m/s/s
 * @param [in] data - pointer to external data structure
 * @retval N/A
 ******************************************************************************/
void  GetAccelData_mPerSecSq(double *data);

/** ****************************************************************************
 * @name GetRateData_radPerSec
 * @brief Get scaled rate sesnors data in rad/ses
 * @param [in] data - pointer to external data structure
 * @retval N/A
 ******************************************************************************/
void  GetRateData_radPerSec(double *data);

/** ****************************************************************************
 * @name GetRateData_degPerSec
 * @brief Get scaled rate sesnors data in deg/ses
 * @param [in] data - pointer to external data structure
 * @retval N/A
 ******************************************************************************/
void  GetRateData_degPerSec(double *data);

/** ****************************************************************************
 * @name GetMagData_G
 * @brief Get scaled magnetometer data in Gauss
 * @param [in] data - pointer to external data structure
 * @retval N/A
 ******************************************************************************/
void  GetMagData_G(double *data);


/** ****************************************************************************
 * @name GetBoardTempData
 * @brief Get Board Temperature 
 * @param [in] temp - pointer to external data structure
 * @retval N/A
 ******************************************************************************/
void  GetBoardTempData(double *temp); 


void AccelerometerDataReadyIRQ(void);
void MagnetomterDataReadyIRQ(void);
void BeginRateSensorRead(void);
void InitSensors(); 
void ActivateSensors(); 

#endif
