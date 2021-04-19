/*****************************************************************************
 * @file config_fields.c Checking field data per Crossbow Serial Protocol
 *   - Providing validity checks on configuration field data.
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


#include <math.h>
#include <stdint.h>


#include "config_fields.h"
#include "scaling.h"
#include "qmath.h"
//#include "Indices.h"
#include "platformAPI.h"
#include "sensors_data.h"
#include "GpsData.h"
#include "ucb_packet.h"
#include "algorithmAPI.h"

/// proposed configurations
static ConfigurationStruct proposedRamConfiguration;
static ConfigurationStruct proposedEepromConfiguration;

static BOOL portConfigurationChanged = FALSE; // port settings are to be changed
extern BOOL memsicMag;
BOOL fWriteRequest = FALSE;

static uint16_t newOrientation = 0xFFFF;
static uint16_t newAccelFiltr  = 0xFFFF;
static uint16_t newRateFiltr   = 0xFFFF;
static uint16_t newEcuAddress  = 0xFFFF;
static uint16_t newEcuBaudrate = 0xFFFF;
static uint16_t newEcuPacketType      = 0xFFFF;
static uint16_t newEcuPacketRate      = 0xFFFF;

static uint16_t newEcuUartBaudrate = 0xFFFF;
static uint16_t newEcuUartPacketType = 0xFFFF;
static uint16_t newEcuUartPacketRate = 0xFFFF;

static uint16_t tempEcuUartBaudrate = 0xFFFF;
static uint16_t tempEcuUartPacketType = 0xFFFF;
static uint16_t tempEcuUartPacketRate = 0xFFFF;


/// for scaled sensor packet
#define MAX_TEMP_4_SENSOR_PACKET   99.9
#define MAX_OUTPUT_TEMP_q27  1335466394   // iq27( 1335466394 ) =  9.5 [ 10 degC ] =  99.5 [ degC ]
#define MIN_OUTPUT_TEMP_q27  -671088640   // iq27( -671088640 ) = -5.0 [ 10 degC ] = -50.0 [ degC ]

uint16_t GetNewOrientation()
{
    return newOrientation;
}

uint16_t GetNewAccelFiltr()
{
    return newAccelFiltr;
}

uint16_t GetNewRateFiltr()
{
    return newRateFiltr;
}

uint16_t GetNewEcuAddress()
{
    return newEcuAddress;
}

uint16_t GetNewEcuBaudrate()
{
    return newEcuBaudrate;
}

uint16_t GetNewEcuPacketType()
{
    return newEcuPacketType;
}

uint16_t GetNewEcuPacketRate()
{
    return newEcuPacketRate;
}

uint16_t GetNewEcuUartBaudrate()
{
    return newEcuUartBaudrate;
}


uint16_t GetNewEcuUartPacketRate()
{
    return newEcuUartPacketRate;
}



void    ResetChanges()
{
    newOrientation = 0xFFFF;
    newAccelFiltr  = 0xFFFF;
    newRateFiltr   = 0xFFFF;
    newEcuAddress  = 0xFFFF;
    newEcuBaudrate = 0xFFFF;
    newEcuPacketRate     = 0xFFFF;
    newEcuPacketType     = 0xFFFF;

    newEcuUartBaudrate   = 0xFFFF;
    newEcuUartPacketType = 0xFFFF;
    newEcuUartPacketRate = 0xFFFF;
}


/** ****************************************************************************
 * @name CheckPacketRateDivider
 * @brief checks for valid data.
 * @author Darren Liccardo, Jan. 2004
 * @author Dong An, 2007, 2008
 * Trace:
 * [SDD_CHECK_PACKET_RATE_DIVIDER <-- SRC_CHECK_PACKET_RATE_DIVIDER]
 * @param [in] packetRateDivider: the divider for the requested packet rate.
 * @retval 1 if available, zero otherwise.
 ******************************************************************************/
BOOL CheckPacketRateDivider (uint16_t packetRateDivider)
{
    switch (packetRateDivider) {
    case 0:
    case 1:
    case 2:
    case 4:
    case 5:
    case 10:
    case 20:
    case 25:
    case 50:
    case 100:
    case 200:
        return TRUE;
    default:
        return FALSE;
    }
} /* end CheckPacketRateDivider */




/** ****************************************************************************
 * @name checkOrientation
 * @brief verifies the integrity of a proposed orientation field.
 * @author Darren Liccardo, Jan. 2004
 * @author Dong An, 2007, 2008
 * Trace: [SDD_CHK_FIELD_ORIENT<-- SRC_CHECK_ORIENTATION]
 * [SDD_INIT_CONFIGURATION_ORIENT_VALID <-- SRC_CHECK_ORIENTATION]
 * @param [in] orientation: the value of the orientation field to verify.
 * @retval 	one for valid orientations, zero otherwise.
 ******************************************************************************/
BOOL CheckOrientation (uint16_t orientation)
{
    switch ( orientation ) {
    case 0:             // 0000 000 | 00 0 | 00 0 | 00 0 +Z +Y +X
    case 9:             // 0000 000 | 00 0 | 00 1 | 00 1 +Z -Y -X 
    case 35:            // 0000 000 | 00 0 | 10 0 | 01 1 +Z +X +Y
    case 42:            // 0000 000 | 00 0 | 10 1 | 01 0 +Z -X +Y
    case 65:            // 0000 000 | 00 1 | 00 0 | 00 1 -Z +Y -X 
    case 72:            // 0000 000 | 00 1 | 00 1 | 00 0 -Z -Y +X
    case 98:            // 0000 000 | 00 1 | 10 0 | 01 0 -Z +X +Y 
    case 107:           // 0000 000 | 00 1 | 10 1 | 01 1 -Z -X -Y
    case 133:           // 0000 000 | 01 0 | 00 0 | 10 1 +X +Y -Z
    case 140:           // 0000 000 | 01 0 | 00 1 | 10 0 +X -Y +Z
    case 146:           // 0000 000 | 01 0 | 01 0 | 01 0 +X +Z +Y
    case 155:           // 0000 000 | 01 0 | 01 1 | 01 1 +X -Z -Y
    case 196:           // 0000 000 | 01 1 | 00 0 | 10 0 -X +Y +Z
    case 205:           // 0000 000 | 01 1 | 00 1 | 10 1 -X -Y -Z
    case 211:           // 0000 000 | 01 1 | 01 0 | 01 1 -X +Z -Y
    case 218:           // 0000 000 | 01 1 | 01 1 | 01 0 -X -Z +Y
    case 273:           // 0000 000 | 10 0 | 01 0 | 00 1 +Y +Z -X
    case 280:           // 0000 000 | 10 0 | 01 1 | 00 0 +Y -Z +X
    case 292:           // 0000 000 | 10 0 | 10 0 | 10 0 +Y +X +Z
    case 301:           // 0000 000 | 10 0 | 10 1 | 10 1 +Y -X -Z
    case 336:           // 0000 000 | 10 1 | 01 0 | 00 0 -Y +Z +X
    case 345:           // 0000 000 | 10 1 | 01 1 | 00 1 -Y -Z -X
    case 357:           // 0000 000 | 10 1 | 10 0 | 10 1 -Y +X -Z
    case 364:           // 0000 000 | 10 1 | 10 1 | 10 0 -Y -X +Z
        return TRUE;
    default:
        return FALSE;
    }
}  /*end CheckOrientation */


/** ****************************************************************************
 * @name CheckFieldData
 * @brief checks if field data has valid values.
 * @author Dong An, 2007, 2008
 * Trace: [SDD_CHECK_FIELD_DATA <-- SRC_CHECK_FIELD_DATA]
 * [SDD_CHK_FIELD_ADDRESS <-- SRC_CHECK_FIELD_DATA]
 * [SDD_CHK_FIELD_ORIENT  <-- SRC_CHECK_FIELD_DATA]
 * [SDD_CHK_FIELD_BARO   <-- SRC_CHECK_FIELD_DATA]
 * [SDD_CHK_FIELD_PKT_01  <-- SRC_CHECK_FIELD_DATA]
 * [SDD_CHK_FIELD_PKT_02  <-- SRC_CHECK_FIELD_DATA]
 * [SDD_CHK_FIELD_RATE_DIVIDER_01  <-- SRC_CHECK_FIELD_DATA]
 * [SDD_CHK_FIELD_RATE_DIVIDER_02  <-- SRC_CHECK_FIELD_DATA]
 * [SDD_CHK_FIELD_PORT_USE_01  <-- SRC_CHECK_FIELD_DATA]
 * [SDD_CHK_FIELD_PORT_USE_02  <-- SRC_CHECK_FIELD_DATA]
 * [SDD_CHK_FIELD_PORT_RATE_01  <-- SRC_CHECK_FIELD_DATA]
 * [SDD_CHK_FIELD_PORT_RATE_02  <-- SRC_CHECK_FIELD_DATA]
 *
 * @param [in] currentConfiguration - current configuration to modify
 * @param [in] numFields - number of fields to check
 * @author Dong An, 2007, 2008
 * Trace: [SDD_CHECK_FIELD_DATA <-- SRC_CHECK_FIELD_DATA]
 *							index-wise to the array of field IDs
 * @param [out] validFields[] - array of fieldId that have been validated.
 * @retval 	number of valid fields returned in validFields[]
 ******************************************************************************/
static uint8_t CheckFieldData (ConfigurationStruct *currentConfiguration,
                               uint8_t             numFields,
                               uint16_t            fieldId [],
                               uint16_t            fieldData [],
                               uint16_t            validFields [])
{
    BOOL                packetCodeChanged        = FALSE;
    BOOL                packetRateDividerChanged = FALSE;
    BOOL                userBaudChanged          = FALSE;
    BOOL                uartCongifValid          = FALSE;
    /// index for stepping through proposed configuration fields
    uint8_t             fieldIndex      = 0;
    uint8_t             validFieldIndex = 0; ///< index for building valid return array
    uint8_t             type [UCB_PACKET_TYPE_LENGTH]; ///< working packet type byte buffer
    UcbPacketType       continuousPacketType;
    ConfigurationStruct proposedPortConfig;

    tempEcuUartBaudrate   = 0xFFFF;
    tempEcuUartPacketType = 0xFFFF;
    tempEcuUartPacketRate = 0xFFFF;

    /// copy current configuration - for testing validity of port configuration only
    proposedPortConfig = *currentConfiguration;

    /// update new field settings in proposed configuration */
    for (fieldIndex = 0; fieldIndex < numFields; ++fieldIndex) {
        if ((fieldId[fieldIndex] >= LOWER_CONFIG_ADDR_BOUND) &&
            (fieldId[fieldIndex] <= UPPER_CONFIG_ADDR_BOUND)) {
            /// parse field ID and, if applicable, check using respective
            /// function (not all fields require this)
            switch (fieldId[fieldIndex]) {
                case PACKET_TYPE_FIELD_ID:
                    /// get enum for requested continuous packet type
                    type[0] = (uint8_t)((fieldData[fieldIndex] >> 8) & 0xff);
                    type[1] = (uint8_t)(fieldData[fieldIndex] & 0xff);

                    continuousPacketType = UcbPacketBytesToPacketType(type);

                    /// check that a valid continuous output packet has been selected
                    if (UcbPacketIsAnOutputPacket(continuousPacketType) == TRUE) {
                        packetCodeChanged             = TRUE;
                        proposedPortConfig.packetCode = fieldData[fieldIndex];
                        if(fWriteRequest){
                            tempEcuUartPacketType = fieldData[fieldIndex];
                        }
                    }
                    break;
                case PACKET_RATE_DIVIDER_FIELD_ID:
                    packetRateDividerChanged             = TRUE;
                    proposedPortConfig.packetRateDivider = fieldData[fieldIndex];
                    if(fWriteRequest){
                        tempEcuUartPacketRate = fieldData[fieldIndex];
                    }
                    break;
                case BAUD_RATE_USER_ID:
                    userBaudChanged = TRUE;
                    proposedPortConfig.baudRateUser = fieldData[fieldIndex];
                    if(fWriteRequest){
                        tempEcuUartBaudrate = fieldData[fieldIndex];
                    }
                    break;
                case ORIENTATION_FIELD_ID:
                    if (CheckOrientation(fieldData[fieldIndex]) == TRUE) {
                        /// update proposed configuration
                        currentConfiguration->orientation.all = fieldData[fieldIndex];
                        /// add to valid list
                        validFields[validFieldIndex++]        = fieldId[fieldIndex];
                        // Set the flags to RESTART the algorithm
                        InitializeAlgorithmStruct(getAlgorithmFrequency(),CurrentIMU);
                        if(fWriteRequest){
                            newOrientation = fieldData[fieldIndex];
                        }
                    }
                    break;
                case ACCEL_FILTR_FIELD_ID:
                    ((uint16_t *)currentConfiguration)[(fieldId[fieldIndex])] = fieldData[fieldIndex];
                    validFields[validFieldIndex++] = fieldId[fieldIndex];
                    if(fWriteRequest){
                        newAccelFiltr = fieldData[fieldIndex];
                    }
                    break;
                case RATE_FILTR_FIELD_ID:
                    ((uint16_t *)currentConfiguration)[(fieldId[fieldIndex])] = fieldData[fieldIndex];
                    validFields[validFieldIndex++] = fieldId[fieldIndex];
                    if(fWriteRequest){
                        newRateFiltr = fieldData[fieldIndex];
                    }
                    break;
                case ECU_ADDRESS_FIELD_ID:
                    ((uint16_t *)currentConfiguration)[(fieldId[fieldIndex])] = fieldData[fieldIndex];
                    validFields[validFieldIndex++] = fieldId[fieldIndex];
                    if(fWriteRequest){
                        newEcuAddress = fieldData[fieldIndex];
                    }
                    break;
                case ECU_BAUD_RATE_FIELD_ID:
                    ((uint16_t *)currentConfiguration)[(fieldId[fieldIndex])] = fieldData[fieldIndex];
                    validFields[validFieldIndex++] = fieldId[fieldIndex];
                    if(fWriteRequest){
                        newEcuBaudrate = fieldData[fieldIndex];
                    }
                    break;
                case ECU_PACKET_RATE_FIELD_ID:
                    ((uint16_t *)currentConfiguration)[(fieldId[fieldIndex])] = fieldData[fieldIndex];
                    validFields[validFieldIndex++] = fieldId[fieldIndex];
                    if(fWriteRequest){
                        newEcuPacketRate = fieldData[fieldIndex];
                    }
                    break;
                case ECU_PACKET_TYPE_FIELD_ID:
                    ((uint16_t *)currentConfiguration)[(fieldId[fieldIndex])] = fieldData[fieldIndex];
                    validFields[validFieldIndex++] = fieldId[fieldIndex];
                    if(fWriteRequest){
                        newEcuPacketType = fieldData[fieldIndex];
                    }
                    break;
                case OFFSET_ROLL_ALIGN_FIELD_ID:
                    //int16_t tmp = (int16_t)(fieldData[fieldIndex]);
                    break;
                case OFFSET_PITCH_ALIGN_FIELD_ID:
                    //tmp = (int16_t)(fieldData[fieldIndex]);
                    break;
                case OFFSET_YAW_ALIGN_FIELD_ID:
                    //tmp = (int16_t)(fieldData[fieldIndex]);
                    break;
                default:
                    ((uint16_t *)currentConfiguration)[(fieldId[fieldIndex])] = fieldData[fieldIndex];
                    validFields[validFieldIndex++] = fieldId[fieldIndex];
                    break;
            }
        }
    }
    /// check proposed port configuration field settings (order/priority matters!)
    if (userBaudChanged == TRUE) {
        if (ValidPortConfiguration(&proposedPortConfig) == TRUE) {
            portConfigurationChanged = TRUE;
            uartCongifValid          = TRUE;
            /// add configuration changes to proposed configuration and add all
            /// relevant fields to valid list
            if (packetCodeChanged == TRUE) {
                currentConfiguration->packetCode = proposedPortConfig.packetCode;
                validFields[validFieldIndex++]   = PACKET_TYPE_FIELD_ID;
            }

            if (packetRateDividerChanged == TRUE) {
                currentConfiguration->packetRateDivider = proposedPortConfig.packetRateDivider;
                validFields[validFieldIndex++]          = PACKET_RATE_DIVIDER_FIELD_ID;
            }

            if (userBaudChanged == TRUE) {
                currentConfiguration->baudRateUser = proposedPortConfig.baudRateUser;
                validFields[validFieldIndex++]     = BAUD_RATE_USER_ID;
            }
        }
    } else if ((packetCodeChanged == TRUE) ||
               (packetRateDividerChanged == TRUE)) {
        /// port usage or baud settings haven't changed, DON'T indicate port
        /// configuration change
        proposedPortConfig.baudRateUser = currentConfiguration->baudRateUser;

        if (ValidPortConfiguration(&proposedPortConfig) == TRUE) {
            uartCongifValid       = TRUE;
            if (packetCodeChanged == TRUE) {
                currentConfiguration->packetCode = proposedPortConfig.packetCode;
                validFields[validFieldIndex++]   = PACKET_TYPE_FIELD_ID;
            }

            if (packetRateDividerChanged == TRUE) {
                currentConfiguration->packetRateDivider = proposedPortConfig.packetRateDivider;
                validFields[validFieldIndex++]          = PACKET_RATE_DIVIDER_FIELD_ID;
            }
        }
    }

    if(fWriteRequest && uartCongifValid){
        if(tempEcuUartBaudrate != 0xFFFF){
            newEcuUartBaudrate = tempEcuUartBaudrate;
        }
        if(tempEcuUartPacketType != 0xFFFF){
            newEcuUartPacketType = tempEcuUartPacketType;
        }
        if(tempEcuUartPacketRate != 0xFFFF){
            newEcuUartPacketRate = tempEcuUartPacketRate;
        }
    }

    fWriteRequest         = FALSE;

    return validFieldIndex;
} /* end CheckFieldData */

/** ****************************************************************************
 * @name CheckRamFieldData
 * @brief checks if field data has valid values.
 * Trace: [SDD_CHK_RAM_FIELDS_02 <-- SRC_CHECK_RAM_FIELD_DATA]
 * @param [in] numFields - number of fields to check
 * @param [in] fieldId[] - array of field IDs.
 * @param [in] fieldData[] - array of field data corresponding
 *							index-wise to the array of field IDs
 * @param [out] validFields[] - array of fieldId that have been validated.
 * @retval number of valid fields returned in validFields[]
 ******************************************************************************/
uint8_t CheckRamFieldData (uint8_t  numFields,
                           uint16_t fieldId [],
                           uint16_t fieldData [],
                           uint16_t validFields [])
{
    proposedRamConfiguration = gConfiguration; /// current RAM configuration

    return CheckFieldData(&proposedRamConfiguration,
                          numFields,
                          fieldId,
                          fieldData,
                          validFields);
}

/** ****************************************************************************
 * @name CheckEepromFieldData
 * @brief checks if field data has valid values.
 * Trace: [SDD_CHK_EEPROM_FIELDS_02 <-- SRC_CHECK_EEPROM_FIELD_DATA]
 * @param [in] numFields - number of fields to check
 * @param [in] fieldId[] - array of field IDs.
 * @param [in] fieldData[] - array of field data corresponding
 *							index-wise to the array of field IDs
 * @param [out] validFields[] - array of fieldId that have been validated.
 * @retval number of valid fields returned in validFields[]
 ******************************************************************************/
uint8_t CheckEepromFieldData (uint8_t  numFields,
                              uint16_t fieldId [],
                              uint16_t fieldData [],
                              uint16_t validFields [])
{   /// copy current EEPROM configuration
    readUnitConfigurationStruct(&proposedEepromConfiguration);
    fWriteRequest = TRUE;
    return CheckFieldData(&proposedEepromConfiguration,
                          numFields,
                          fieldId,
                          fieldData,
                          validFields);
}

/** ****************************************************************************
 * @name SetFieldData
 * @brief Perform config changes required by the "SF" command
 * Trace: [SDD_UCB_SET_FIELD_01 <-- SRC_SET_FIELD_DATA]
 *	[SDD_UCB_SET_FIELD_02 <-- SRC_SET_FIELD_DATA]
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void SetFieldData (void)
{
    /// special handling for changing port configuration
    if (portConfigurationChanged == TRUE) {
        /// assign proposed configuration to actual configuration
        gConfiguration = proposedRamConfiguration;
        portConfigurationChanged = FALSE; /// reset changed flag
    } else {  /// non-port related configuration change
        /// assign proposed configuration to actual configuration
        gConfiguration = proposedRamConfiguration;
    }
} /* end SetFieldData */

/** ****************************************************************************
 * @name WriteFieldData
 * @brief write the data from the WF command into eeprom
 * Trace: [SDD_UCB_WRITE_FIELD <-- SRC_WRITE_FIELD_DATA]
 * @param N/A
 * @retval status TRUE, FALSE
 ******************************************************************************/
BOOL WriteFieldData (void)
{
    BOOL     success;

    uint16_t *ptr = (uint16_t*) &proposedEepromConfiguration;
    
    /// write entire proposed configuration back to EEPROM
    if (saveUnitConfigurationStruct(ptr) == 0) {
        success = TRUE;
    } else {
        success = FALSE;
    }

    return success;
} /* end WriteFieldData */


/** ****************************************************************************
 * @name appendCorrectedRates
 * @brief calculates the algorithm corrected angular rates and formats them for
 *        output.
 * @author Darren Liccardo, Aug. 2005
 * @author Dong An, 2007,2008
 * Trace: [SDD_APPEND_CORRECTED_RATES <-- SRC_APPEND_CORRECTED_RATES]
 * @param [in] response - points to the beginning of the packet array.
 * @param [in] index - response[index] is where data is added.
 * @retval  modified index to next avaliable response buffer location.
 ******************************************************************************/
#include "EKF_Algorithm.h"
#define TEST_OUT_PCKT
#ifndef TEST_OUT_PCKT //XBOW_ALGORITHM
uint16_t appendCorrectedRates (uint8_t  *response, uint16_t index)
{
    int16_t tmp = 0;
    real correctedRate_B[3];

    // retrieve corrected rates here
    GetCorrectedRates(correctedRate_B);

    // X-Axis (i=0)
    tmp = (int16_t) SCALE_BY_2POW16_OVER_7PI(correctedRate_B[X_AXIS]);
    index = uint16ToBuffer(response, index, tmp);
    // Y-AXIS (i=1)
    tmp = (int16_t) SCALE_BY_2POW16_OVER_7PI(correctedRate_B[Y_AXIS]);
    index = uint16ToBuffer(response, index, tmp);
    // Z-Axis (i=2)
    tmp = (int16_t) SCALE_BY_2POW16_OVER_7PI(correctedRate_B[Z_AXIS]);
    index = uint16ToBuffer(response, index, tmp);

    return index;
}
#else
uint16_t appendCorrectedRates (uint8_t  *response,
                               uint16_t index)
{
    int16_t tmp;

    /// i = 0
    tmp = (int16_t) SCALE_BY_2POW16_OVER_7PI(gKalmanFilter.correctedRate_B[X_AXIS]);
    index = uint16ToBuffer(response,
                           index,
                           tmp);
    /// i = 1
    tmp = (int16_t) SCALE_BY_2POW16_OVER_7PI(gKalmanFilter.correctedRate_B[Y_AXIS]);
    index = uint16ToBuffer(response,
                           index,
                           tmp);
    /// i = 2
    tmp = (int16_t) SCALE_BY_2POW16_OVER_7PI(gKalmanFilter.correctedRate_B[Z_AXIS]);
    index = uint16ToBuffer(response,
                           index,
                           tmp);
    return index;
}  
/* end appendCorrectedRates */


#endif


/** ****************************************************************************
 * @name appendRates
 * @brief calculates the algorithm corrected angular
 *    rates and formats them for output.
 * @author Darren Liccardo, Aug. 2005
 * @author Dong An, 2007,2008
 * Trace: [SDD_APPEND_RATES <-- SRC_APPEND_RATES]
 * @param [in] response - points to the beginning of the packet array.
 * @param [in] index - response[index] is where data is added.
 * @retval  modified index to next avaliable response buffer location.
 ******************************************************************************/
uint16_t appendRates (uint8_t  *response,
                      uint16_t index)
{
    int tmp;

    /// X-Axis
    tmp = _qmul( TWO_POW16_OVER_7PI_q19, gSensorsData.scaledSensors_q27[XRATE], 19, 27, 16) >> 16;
    index = uint16ToBuffer(response, index, (uint16_t)tmp);
    /// Y-Axis
    tmp = _qmul( TWO_POW16_OVER_7PI_q19, gSensorsData.scaledSensors_q27[YRATE], 19, 27, 16) >> 16;
    index = uint16ToBuffer(response, index, (uint16_t)tmp);
    /// Z-Axis
    tmp = _qmul( TWO_POW16_OVER_7PI_q19, gSensorsData.scaledSensors_q27[ZRATE], 19, 27, 16) >> 16;
    index = uint16ToBuffer(response, index,  (uint16_t)tmp);
    return index;
} /* end appendRates */

/** ****************************************************************************
 * @name appendMagReadings
 * @brief calculates the algorithm corrected magnetometer readings and formats
 *        them for output.
 * @author Darren Liccardo, Aug. 2005
 * @author Dong An, 2007,2008
 * @author Joe Motyka, June 2013
 * Trace: [SDD_APPEND_MAG_READINGS <-- SRC_APPEND_MAG_READINGS]
 * @param [in] response - points to the beginning of the packet array.
 * @param [in] index - response[index] is where data is added.
 * @retval  modified index to next avaliable response buffer location.
 ******************************************************************************/
uint16_t appendMagReadings( uint8_t  *response,
                            uint16_t index )
{
    int tmp;

    if(!memsicMag){

    /// Cycle through each magnetometer axis and convert it to a 16-bit integer
    /// X-Axis
    tmp = _qmul( TWO_POW16_OVER_20_q19, gSensorsData.scaledSensors_q27[XMAG], 19, 27, 16 ) >> 16;
    /// Split the 16-bit integer into two 8-bit numbers and place it into the buffer
    index = uint16ToBuffer(response,
                           index,
                           tmp);
    /// Y-Axis
    tmp = _qmul( TWO_POW16_OVER_20_q19, gSensorsData.scaledSensors_q27[YMAG], 19, 27, 16 ) >> 16;
    index = uint16ToBuffer(response,
                           index,
                           tmp);
    /// Z-Axis
    tmp = _qmul( TWO_POW16_OVER_20_q19, gSensorsData.scaledSensors_q27[ZMAG], 19, 27, 16 ) >> 16;
    index = uint16ToBuffer(response,
                           index,
                           tmp);
    }else{
        for(int i = 0; i < 3; i++){
            tmp   = gSensorsData.scaledSensors[XMAG + i] * 3276.8;
            index = uint16ToBuffer(response, index, tmp);
        }
    }

    return index;
} /* end appendMagReadings */

/** ****************************************************************************
 * @name appendTangentRates
 * @brief formats the local level frame angular rates for output.
 * Trace: [SDD_APPEND_TANGENT_RATES <-- SRC_APPEND_TANGENT_RATES]
 * @param [in] response - points to the beginning of the packet array.
 * @param [in] index - response[index] is where data is added.
 * @retval  modified index to next avaliable response buffer location.
 ******************************************************************************/
/*
uint16_t appendTangentRates (uint8_t  *response,
                             uint16_t index)
{
    int tmp;

    /// X-axis
    tmp = (int)(SCALE_BY_2POW16_OVER_7PI(gAlgorithm.tangentRates[X_AXIS]));
    index = uint16ToBuffer(response,
                           index,
                           tmp);
    /// Y-axis
    tmp = (int)(SCALE_BY_2POW16_OVER_7PI(gAlgorithm.tangentRates[Y_AXIS]));
    index = uint16ToBuffer(response,
                           index,
                           tmp);
    /// Z-axis
    tmp = (int)(SCALE_BY_2POW16_OVER_7PI(gAlgorithm.tangentRates[Z_AXIS]));
    index = uint16ToBuffer(response,
                           index,
                           tmp);
    return index;
}*/ 
/* end appendTangentRates */


/** ****************************************************************************
 * @name appendAccels
 * @brief calculates the algorithm corrected accelerations and formats them for
 *        output.
 * @author Darren Liccardo, Aug. 2005
 * @author Dong An, 2007,2008
 * Trace: [SDD_APPEND_ACCELS <-- SRC_APPEND_ACCELS]
 * @param [in] response - points to the beginning of the packet array.
 * @param [in] index - response[index] is where data is added.
 * @retval  modified index to next avaliable response buffer location.
 ******************************************************************************/
uint16_t appendAccels (uint8_t  *response,
                       uint16_t index)
{
    uint16_t tmp;
   

    /// X-Axis (i=0)
    tmp = _qmul( TWO_POW16_OVER_20_q19, gSensorsData.scaledSensors_q27[XACCEL], 19, 27, 16 ) >> 16;
    index = uint16ToBuffer(response, index, tmp);
    /// Y-Axis (i=1)
    tmp = _qmul( TWO_POW16_OVER_20_q19, gSensorsData.scaledSensors_q27[YACCEL], 19, 27, 16 ) >> 16;
    //tmp = (int)(SCALE_BY_2POW16_OVER_20(uncorrectedAccel_B[Z_AXIS]));
    index = uint16ToBuffer(response, index, tmp);
    /// Z-Axis (i=2)
    tmp = _qmul( TWO_POW16_OVER_20_q19, gSensorsData.scaledSensors_q27[ZACCEL], 19, 27, 16 ) >> 16;
    index = uint16ToBuffer(response, index, tmp);
    
    return index;
} /* end appendAccels */

/** ****************************************************************************
 * @name appendCorrectedAccels
 * @brief calculates the algorithm corrected accelerations and formats them for
 *        output.
 * @author Darren Liccardo, Aug. 2005
 * @author Dong An, 2007,2008
 * Trace: [SDD_APPEND_ACCELS <-- SRC_APPEND_ACCELS]
 * @param [in] response - points to the beginning of the packet array.
 * @param [in] index - response[index] is where data is added.
 * @retval  modified index to next avaliable response buffer location.
 ******************************************************************************/
#ifdef XBOW_ALGORITHM
uint16_t appendCorrectedAccels (uint8_t  *response, uint16_t index)
{
    uint16_t tmp = 0;
    real correctedAccel_B[3];

    // Retrieve corrected accels here
    GetEKF_CorrectedAccels(correctedAccel_B);

    // X-Axis (i=0)
    tmp = (int)SCALE_BY_2POW16_OVER_20(correctedAccel_B[X_AXIS]);
    index = uint16ToBuffer(response, index, tmp);
    // Y-Axis (i=1)
    tmp = (int)SCALE_BY_2POW16_OVER_20(correctedAccel_B[Y_AXIS]);
    index = uint16ToBuffer(response, index, tmp);
    // Z-Axis (i=2)
    tmp = (int)SCALE_BY_2POW16_OVER_20(correctedAccel_B[Z_AXIS]);
    index = uint16ToBuffer(response, index, tmp);

    return index;
}
#else
uint16_t appendCorrectedAccels (uint8_t  *response,
                                uint16_t index)
{
    uint16_t tmp;

    /// X-Axis
    tmp = _qmul( TWO_POW16_OVER_20_q19,
                 (int32_t)(gKalmanFilter.correctedAccel_B[XACCEL]*13681725.58613659),
                 19, 27, 16 ) >> 16;
    index = uint16ToBuffer(response,
                           index,
                           tmp);
    /// Y-Axis
    tmp = _qmul( TWO_POW16_OVER_20_q19,
                 (int32_t)(gKalmanFilter.correctedAccel_B[YACCEL]*13681725.58613659),
                 19, 27, 16 ) >> 16;
    index = uint16ToBuffer(response,
                           index,
                           tmp);
    /// Z-Axis
    tmp = _qmul( TWO_POW16_OVER_20_q19,
                 (int32_t)(gKalmanFilter.correctedAccel_B[ZACCEL]*13681725.58613659),
                 19, 27, 16 ) >> 16;
    index = uint16ToBuffer(response,
                           index,
                           tmp);
    return index;
} /* end appendAccels */
#endif

/** ****************************************************************************
 * @name appendTangentAccels
 * @brief calculates Along Heading Acceleration, Cross Heading Acceleration and
 *        Vertical Acceleration for ARINC705, and formats them for output.
 * Trace: [SDD_APPEND_ACCELS <-- SRC_APPEND_ACCELS]
 * @param [in] response - points to the beginning of the packet array.
 * @param [in] index - response[index] is where data is added.
 * @retval  modified index to next avaliable response buffer location.
 ******************************************************************************/
/*
uint16_t appendTangentAccels (uint8_t  *response,
                              uint16_t index)
{
    int    tmp;
    double aHcHvAccel[3];

    aHcHvAccel[X_AXIS] =   gAlgorithm.tangentAccels[X_AXIS];
    aHcHvAccel[Y_AXIS] =   gAlgorithm.tangentAccels[Y_AXIS];
    aHcHvAccel[Z_AXIS] = -(gAlgorithm.tangentAccels[Z_AXIS] + 1.0);

    /// X-Axis
    tmp = (int)( SCALE_BY_2POW16_OVER_20( aHcHvAccel[X_AXIS] ) );
    index = uint16ToBuffer(response,
                           index,
                           tmp);
    /// Y-Axis
    tmp = (int)( SCALE_BY_2POW16_OVER_20( aHcHvAccel[Y_AXIS] ) );
    index = uint16ToBuffer(response,
                           index,
                           tmp);
    /// Z-Axis
    tmp = (int)( SCALE_BY_2POW16_OVER_20( aHcHvAccel[Z_AXIS] ) );
    index = uint16ToBuffer(response,
                           index,
                           tmp);
    return index;
}*/ 
/* end appendTangentAccels */


/** ****************************************************************************
 * @name appendRateTemp
 * @brief formats rate and board temperature data for output.
 * @author Darren Liccardo, Dec. 2005
 * @author Dong An, 2007,2008
 * Trace: [SDD_APPEND_TEMPS_01 <-- SRC_APPEND_TEMPS]
 * [SDD_APPEND_TEMPS_02 <-- SRC_APPEND_TEMPS]
 * @param [in] response - points to the beginning of the packet array.
 * @param [in] index - response[index] is where data is added.
 * @retval  modified index to next avaliable response buffer location.
 ******************************************************************************/
uint16_t appendRateTemp (uint8_t  *response,
                         uint16_t index)
{
    uint16_t tmp;
    double   tmpD;

    tmpD = gSensorsData.scaledSensors[XRTEMP];

    if (tmpD >= MAX_TEMP_4_SENSOR_PACKET) {
        tmpD =  MAX_TEMP_4_SENSOR_PACKET;
    }
    if (tmpD <= -MAX_TEMP_4_SENSOR_PACKET) {
        tmpD =  -MAX_TEMP_4_SENSOR_PACKET;
    }

    tmp   = (int)(SCALE_BY_2POW16_OVER_200(tmpD));

    index = uint16ToBuffer(response,
                           index,
                           tmp);
    return index;

} /*end appendTemps */


/** ****************************************************************************
 * @name appendTemps
 * @brief formats rate and board temperature data for output.
 * @author Darren Liccardo, Dec. 2005
 * @author Dong An, 2007,2008
 * Trace: [SDD_APPEND_TEMPS_01 <-- SRC_APPEND_TEMPS]
 * [SDD_APPEND_TEMPS_02 <-- SRC_APPEND_TEMPS]
 * @param [in] response - points to the beginning of the packet array.
 * @param [in] index - response[index] is where data is added.
 * @retval  modified index to next avaliable response buffer location.
 ******************************************************************************/
uint16_t appendTemps (uint8_t  *response,
                      uint16_t index)
{
    uint16_t tmp;

    // Rate sensor temperature hn units of [ 10 degC ] but the output must be in
    //   degC scaled by 2^16/200 so the multiplier must be ( 2^16/20 ). The max
    //   temp should be 15.5 [ 10 degC ], which places the max value of l
    // iqmath
    if( gSensorsData.scaledSensors_q27[XRTEMP+0] >= MAX_OUTPUT_TEMP_q27 ) {
        gSensorsData.scaledSensors_q27[XRTEMP+0] = MAX_OUTPUT_TEMP_q27;
    } else if( gSensorsData.scaledSensors_q27[XRTEMP+0] <= MIN_OUTPUT_TEMP_q27 ) {
        gSensorsData.scaledSensors_q27[XRTEMP+0] = MIN_OUTPUT_TEMP_q27;
    }

    // Convert to scaled output T { degC ] * ( 2^16/200 )
    tmp = _qmul( TWO_POW16_OVER_20_q19, gSensorsData.scaledSensors_q27[XRTEMP+0], 19, 27, 15 ) >> 15;
    index = uint16ToBuffer(response,
                           index,
                           tmp);

    // iqmath
    if( gSensorsData.scaledSensors_q27[XRTEMP+1] >= MAX_OUTPUT_TEMP_q27 ) {
        gSensorsData.scaledSensors_q27[XRTEMP+1] = MAX_OUTPUT_TEMP_q27;
    } else if( gSensorsData.scaledSensors_q27[XRTEMP+1] <= MIN_OUTPUT_TEMP_q27 ) {
        gSensorsData.scaledSensors_q27[XRTEMP+1] = MIN_OUTPUT_TEMP_q27;
    }

    tmp = _qmul( TWO_POW16_OVER_20_q19, gSensorsData.scaledSensors_q27[XRTEMP+1], 19, 27, 15 ) >> 15;
    index = uint16ToBuffer(response,
                           index,
                           tmp);

    // iqmath
    if( gSensorsData.scaledSensors_q27[XRTEMP+2] >= MAX_OUTPUT_TEMP_q27 ) {
        gSensorsData.scaledSensors_q27[XRTEMP+2] = MAX_OUTPUT_TEMP_q27;
    } else if( gSensorsData.scaledSensors_q27[XRTEMP+2] <= MIN_OUTPUT_TEMP_q27 ) {
        gSensorsData.scaledSensors_q27[XRTEMP+2] = MIN_OUTPUT_TEMP_q27;
    }
    tmp = _qmul( TWO_POW16_OVER_20_q19, gSensorsData.scaledSensors_q27[XRTEMP+2], 19, 27, 15 ) >> 15;
    index = uint16ToBuffer(response,
                           index,
                           tmp);

    // Board temperature is in units of [ 10 degC ] but the output must be in
    //   degC scaled 2^16/200 so the multiplier must be ( 2^16/20 ). The max
    //   temp should be 15.5 [ 10 degC ], which places the max value of l
    // iqmath
    if( gSensorsData.scaledSensors_q27[BTEMP] >= MAX_OUTPUT_TEMP_q27 ) {
        gSensorsData.scaledSensors_q27[BTEMP] = MAX_OUTPUT_TEMP_q27;
    } else if( gSensorsData.scaledSensors_q27[BTEMP] <= MIN_OUTPUT_TEMP_q27 ) {
        gSensorsData.scaledSensors_q27[BTEMP] = MIN_OUTPUT_TEMP_q27;
    }

    tmp = _qmul( TWO_POW16_OVER_20_q19, gSensorsData.scaledSensors_q27[BTEMP], 19, 27, 15 ) >> 15;
    // Split the 16-bit integer into two 8-bit numbers and place it into the response array
    index = uint16ToBuffer(response,
                           index,
                           tmp);
    return index;
} /*end appendTemps */


#include "EKF_Algorithm.h"
/** ****************************************************************************
 * @name appendAttitudeTrue
 * @brief calculates roll, pitch, and true heading and formats them for output.
 * @author Darren Liccardo, Dec. 2005
 * @author Dong An, 2007,2008
 * Trace: [SDD_APPEND_ATTITUDE_TRUE <-- SRC_APPEND_ATTITUDE_TRUE]
 * [SDD_APPEND_TEMPS_02 <-- SRC_APPEND_TEMPS]
 * @param [in] response - points to the beginning of the packet array.
 * @param [in] index - response[index] is where data is added.
 * @retval  modified index to next avaliable response buffer location.
 ******************************************************************************/
#ifdef XBOW_ALGORITHM
uint16_t appendAttitudeTrue (uint8_t  *response, uint16_t index)
{
    int16_t tmp = 0;
    real   eulerAngles[3];

    // retrieve attitude calculated by user-algorithm here
    GetEulerAngles(eulerAngles);

    // X-Axis (angle in radians)
    tmp = (int16_t) SCALE_BY_2POW16_OVER_2PI(eulerAngles[ROLL]);
    index = uint16ToBuffer(response, index, tmp);
    // Y-Axis (angle in radians)
    tmp = (int16_t) SCALE_BY_2POW16_OVER_2PI(eulerAngles[PITCH]);
    index = uint16ToBuffer(response, index, tmp);
    // Z-Axis (angle in radians)
    tmp = (int16_t) SCALE_BY_2POW16_OVER_2PI(eulerAngles[YAW]);
    index = uint16ToBuffer(response, index, tmp);

    return index;
}
#else
uint16_t appendAttitudeTrue (uint8_t  *response,
                             uint16_t index)
{
    int16_t tmp;

    // X-Axis (angle in radians
    tmp = (int16_t) SCALE_BY_2POW16_OVER_2PI(gKalmanFilter.eulerAngles[ROLL]);
    index = uint16ToBuffer(response,
                           index,
                           tmp);
    /// Y-Axis
    tmp = (int16_t) SCALE_BY_2POW16_OVER_2PI(gKalmanFilter.eulerAngles[PITCH]);
    index = uint16ToBuffer(response,
                           index,
                           tmp);
    /// Z-Axis
    tmp = (int16_t) SCALE_BY_2POW16_OVER_2PI(gKalmanFilter.eulerAngles[YAW]);
    index = uint16ToBuffer(response,
                           index,
                           tmp);
    return index;
} /* end appendAttitudeTrue */
#endif

/** ****************************************************************************
 * @name appendInertialCounts
 * @briefformats accels and rates for output.
 * @author Dong An, 2007,2008
 * Trace: [SDD_APPEND_INERTIAL_COUNTS <-- SRC_APPEND_INERTIAL_COUNTS]
 * @param [in] response - points to the beginning of the packet array.
 * @param [in] index - response[index] is where data is added.
 * @retval  modified index to next avaliable response buffer location.
 ******************************************************************************/
uint16_t appendInertialCounts (uint8_t  *response,
                               uint16_t index)
{
    /// X-Axis Accelerometer
    index = uint32ToBuffer(response,
                           index,
                           gSensorsData.rawSensors[XACCEL]);
    /// Y-Axis Accelerometer
    index = uint32ToBuffer(response,
                           index,
                           gSensorsData.rawSensors[YACCEL]);

    /// Z-Axis Accelerometer
    index = uint32ToBuffer(response,
                           index,
                           gSensorsData.rawSensors[ZACCEL]);

    /// X-Axis Rate-Sensor
    index = uint32ToBuffer(response,
                           index,
                           gSensorsData.rawSensors[XRATE]);

    /// Y-Axis Rate-Sensor
    index = uint32ToBuffer(response,
                           index,
                           gSensorsData.rawSensors[YRATE]);

    /// Z-Axis Rate-Sensor
    index = uint32ToBuffer(response,
                           index,
                           gSensorsData.rawSensors[ZRATE]);

    return index;
} /* end appendInertialCounts */

/** ****************************************************************************
 * @name appendMagnetometerCounts
 * @brief formats magnetometers for output.
 * @author Joe Motyka, 2013
 * Trace: [SDD_APPEND_MAGNETOMETER_COUNTS <-- SRC_APPEND_MAGNETOMETER_COUNTS]
 * @param [in] response - points to the beginning of the packet array.
 * @param [in] index - response[index] is where data is added.
 * @retval  modified index to next avaliable response buffer location.
 ******************************************************************************/
uint16_t appendMagnetometerCounts (uint8_t  *response,
                                   uint16_t index)
{
    /// X-Axis
    index = uint32ToBuffer(response,
                           index,
                           gSensorsData.rawSensors[XMAG]);
    /// Y-Axis
    index = uint32ToBuffer(response,
                           index,
                           gSensorsData.rawSensors[YMAG]);
    /// Z-Axis
    index = uint32ToBuffer(response,
                           index,
                           gSensorsData.rawSensors[ZMAG]);
    return index;
} /* end appendMagnetometerCounts */

/** ****************************************************************************
 * @name appendAllTempCounts
 * @brief formats the measured temperature counts of gyro, accel and PCB for
 *        output.
 * @author Darren Liccardo, Oct. 2005
 * @author Dong An, 2007,2008
 * Trace: [SDD_APPEND_ALL_TEMP_COUNTS <-- SRC_APPEND_ALL_TEMP_COUNTS]
 * @param [in] response - points to the beginning of the packet array.
 * @param [in] index - response[index] is where data is added.
 * @retval  modified index to next avaliable response buffer location.
 ******************************************************************************/
uint16_t appendAllTempCounts (uint8_t  *response,
                              uint16_t index)
{
    uint32_t temp;

    /// @brief
    /// The older device had a temperature sensor on each of the sensors (prior
    /// to tri-axial device). The DMU380 only has a temperature sensor on the
    /// rate sensor and the board.  To preserve the packet definition used
    /// previously (and to make the packet work with NavView), the first element
    /// of the group of temperature sensors has data while the rest are zero.

    /// accelerometer temperature sensors
    temp = gSensorsData.rawSensors[ XATEMP ];

   /// X-Axis accelerometer temperature
    index = uint32ToBuffer(response,
                           index,
                           temp);

    /// Y-Axis accelerometer temperature (same as x-axis)
    index = uint32ToBuffer(response,
                           index,
                           temp);

    /// Z-Axis accelerometer temperature (same as x-axis)
    index = uint32ToBuffer(response,
                           index,
                           temp);

    /// Rate Sensor temperature values
    temp = gSensorsData.rawSensors[ XRTEMP ];

    /// X-Axis rate-sensor temperature
    index = uint32ToBuffer(response,
                           index,
                           temp);

    /// Y-Axis rate-sensor temperature (same as x-axis)
    index = uint32ToBuffer(response,
                           index,
                           temp);

    /// Z-Axis rate-sensor temperature (same as x-axis)
    index = uint32ToBuffer(response,
                           index,
                           temp);

    /// The last element of the packet contains the board temperature.
    temp = gSensorsData.rawSensors[ BTEMP ];
    index = uint32ToBuffer(response,
                           index,
                           temp);
    return index;
} /* end appendAllTempCounts */

/** ****************************************************************************
 * @name appendGpsVel
 * @brief Add GPS North East and Down velocities to message
 * @author Doug Hiranaka, 2014
 * Trace:
 * @param [in] response - points to the beginning of the packet array.
 * @param [in] index - response[index] is where data is added.
 * @retval  modified index to next avaliable response buffer location.
 ******************************************************************************/
uint16_t appendGpsVel (uint8_t  *response,
                       uint16_t index)
{
    uint16_t temp = 0;

#ifdef GPS
    /// North
    temp = (uint16_t)SCALE_BY_2POW16_OVER_512(gGpsDataPtr->vNed[GPS_NORTH]);
    index = uint16ToBuffer(response,
                           index,
                           temp);
    /// East
    temp = (uint16_t)SCALE_BY_2POW16_OVER_512(gGpsDataPtr->vNed[GPS_EAST]);
    index = uint16ToBuffer(response,
                           index,
                           temp);
    /// Down
    temp = (uint16_t)SCALE_BY_2POW16_OVER_512(gGpsDataPtr->vNed[GPS_DOWN]);
    index = uint16ToBuffer(response,
                           index,
                           temp);
#else
    index = uint16ToBuffer(response, index, temp);  // just fill by zeroes
    index = uint16ToBuffer(response, index, temp);  // just fill by zeroes
    index = uint16ToBuffer(response, index, temp);  // just fill by zeroes
#endif
    return index;
} /* end appendGpsVel */


/** ****************************************************************************
 * @name appendKalmanVel
 * @brief Add North East and Down velocities from the EKF to message
 * @author t. malerich
 * Trace:
 * @param [in] response - points to the beginning of the packet array.
 * @param [in] index - response[index] is where data is added.
 * @retval  modified index to next avaliable response buffer location.
 ******************************************************************************/
#ifndef XBOW_ALGORITHM
uint16_t appendKalmanVel(uint8_t  *response, uint16_t index)
{
    uint16_t temp = 0;

    //float velocities[3];
    // retrieve velocities here
    // getVelocities(velocities);
    //temp = (uint16_t)SCALE_BY_2POW16_OVER_512(velocities[0]);   //GPS_NORTH;
    index = uint16ToBuffer(response, index, temp);
    //temp = (uint16_t)SCALE_BY_2POW16_OVER_512(velocities[1]);   //GPS_EAST;
    index = uint16ToBuffer(response, index, temp);
    //temp = (uint16_t)SCALE_BY_2POW16_OVER_512(velocities[2]);   //GPS_DOWN;
    index = uint16ToBuffer(response, index, temp);

    return index;
}
#else
uint16_t appendKalmanVel(uint8_t  *response,
                         uint16_t index)
{
    /// North
    uint16_t temp = (uint16_t)SCALE_BY_2POW16_OVER_512(gKalmanFilter.Velocity_N[GPS_NORTH]);
    index = uint16ToBuffer(response,
                           index,
                           temp);
    /// East
    temp = (uint16_t)SCALE_BY_2POW16_OVER_512(gKalmanFilter.Velocity_N[GPS_EAST]);
    index = uint16ToBuffer(response,
                           index,
                           temp);
    /// Down
    temp = (uint16_t)SCALE_BY_2POW16_OVER_512(gKalmanFilter.Velocity_N[GPS_DOWN]);
    index = uint16ToBuffer(response,
                           index,
                           temp);
    return index;
} /* end appendKalmanVel */
#endif


/** ****************************************************************************
 * @name appendGpsPos
 * @brief Add GPS Latitude, longitude and Altitude (elevation) to message
 * @author Doug Hiranaka, 2014
 * Trace:
 * @param [in] response - points to the beginning of the packet array.
 * @param [in] index - response[index] is where data is added.
 * @retval  modified index to next avaliable response buffer location.
 ******************************************************************************/
uint16_t appendGpsPos (uint8_t  *response,
                       uint16_t index)
{
  int16_t temp16 = 0;
  int32_t temp32 = 0;

#ifdef GPS
    /// Longitude
    temp32 = (int32_t) SCALE_BY_2POW32_OVER_2PI(gGpsDataPtr->lon * D2R);
    index = uint32ToBuffer(response,
                           index,
                           temp32);
    /// Latitude
    temp32 = (int32_t) SCALE_BY_2POW32_OVER_2PI(gGpsDataPtr->lat * D2R) ;
    index = uint32ToBuffer(response,
                           index,
                           temp32);
    /// Down
    temp16 = (int16_t) SCALE_BY_2POW16_OVER_2POW14(gGpsDataPtr->alt);
    index = uint16ToBuffer(response,
                           index,
                           temp16);
#else
    index = uint32ToBuffer(response, index, temp32);    // just fill with zeroes
    index = uint32ToBuffer(response, index, temp32);
    index = uint16ToBuffer(response, index, temp16);
#endif

    return index;
} /* end appendGpsPos */


/** ****************************************************************************
 * @name appendKalmanPos
 * @brief Add EKF Latitude, longitude and Altitude (elevation) to message
 * @author t. malerich
 * Trace:
 * @param [in] response - points to the beginning of the packet array.
 * @param [in] index - response[index] is where data is added.
 * @retval  modified index to next avaliable response buffer location.
 ******************************************************************************/
#ifndef XBOW_ALGORITHM
uint16_t appendKalmanPos(uint8_t  *response, uint16_t index)
{
    uint32_t temp32 = 0;
    uint16_t temp16 = 0;
    //double llaDeg[3];
    // retrieve position here
    //  getllaDegrees(degrees);
    /// Longitude
    //temp32 = (int32_t) SCALE_BY_2POW32_OVER_2PI(llaDeg[0] * D2R); // longitude
    index = uint32ToBuffer(response, index, temp32);
    /// Latitude
    //temp32 = (int32_t) SCALE_BY_2POW32_OVER_2PI(llaDeg[1] * D2R) ; // lattitude
    index = uint32ToBuffer(response, index, temp32);
    /// altitude
    //temp16 = (int16_t) SCALE_BY_2POW16_OVER_2POW14(llaDeg[2]);    //altitude
    index = uint16ToBuffer(response, index, temp16);

    return index;
}
#else
uint16_t appendKalmanPos(uint8_t  *response,
                         uint16_t index)
{
    /// Longitude
    int32_t temp32 = (int32_t) SCALE_BY_2POW32_OVER_2PI(gKalmanFilter.llaDeg[LON_IDX] * D2R);
    index = uint32ToBuffer(response,
                           index,
                           temp32);
    /// Latitude
    temp32 = (int32_t) SCALE_BY_2POW32_OVER_2PI(gKalmanFilter.llaDeg[LAT_IDX] * D2R) ;
    index = uint32ToBuffer(response,
                           index,
                           temp32);
    /// altitude
    int16_t temp16 = (int16_t) SCALE_BY_2POW16_OVER_2POW14(gKalmanFilter.llaDeg[ALT_IDX]);
    index = uint16ToBuffer(response,
                           index,
                           temp16);
    return index;
} /* end appendKalmanPos */
#endif


/** ****************************************************************************
 * @name uint32ToBuffer
 * @brief formats the input word for output to a byte buffer.
 * @author Douglas Hiranaka, Jul. 2014
 * @param [in] buffer - points to output buffer being loaded.
 * @param [in] index - response[index] is where data is added.
 * @param [in] inWord - data being added.
 * @retval return the incremented index
 ******************************************************************************/
uint32_t
uint32ToBuffer(uint8_t  *buffer,
               uint16_t index,
               uint32_t inWord)
{
    buffer[index++] = (uint8_t)((inWord >> 24) & 0xff);
    buffer[index++] = (uint8_t)((inWord >> 16) & 0xff);
    buffer[index++] = (uint8_t)((inWord >>  8) & 0xff);
    buffer[index++] = (uint8_t)(inWord & 0xff);

  return index;
}

/** ****************************************************************************
 * @name uint16ToBuffer
 * @brief formats the input short integer for output to a byte buffer.
 * @author Douglas Hiranaka, Jul. 2014
 * @param [in] buffer - points to output buffer being loaded.
 * @param [in] index - response[index] is where data is added.
 * @param [in] inWord - data being added.
 * @retval return the incremented index
 ******************************************************************************/
uint32_t
uint16ToBuffer(uint8_t  *buffer,
               uint16_t index,
               uint16_t inShort)
{
    buffer[index++] = (char)((inShort >> 8) & 0xff);
    buffer[index++] = (char)(inShort & 0xff);

  return index;
}

