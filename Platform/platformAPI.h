/** ******************************************************************************
 * @file platformAPI.h API functions for Interfacing with xbow library funcctions
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


#ifndef _PLATFORM_API_H
#define _PLATFORM_API_H
#include <stdint.h>
#include "GlobalConstants.h"

// NEEDS TO BE CHECKED
BOOL eepromLocked(void);
BOOL lockEeprom(void);
BOOL unlockEeprom(void);
void markConfigAsInvalid(void);
BOOL saveEcuAddress(uint16_t* address);
BOOL platformSetPacketRate(int rate, BOOL fApply);
BOOL platformSetBaudRate(int baudRate, BOOL fApply);
BOOL platformSetOrientation(uint16_t *input, BOOL fApply);
int  platformConvertPacketRateDivider(int configParam);
int  platformGetPacketRateDivider();
void platformSetPacketRateDivider(int div);
int  platformGetOrientation(void);
int  platformGetPacketRate(void);
int  platformGetBaudRate(void);
int  platformGetAccelLfpFreq();
int  platformGetRateLfpFreq();
BOOL appStartedFirstTime(void);
BOOL userApplicationActive(void);
BOOL platformSetOutputPacketCode(uint16_t code, BOOL fApply);
BOOL platformSelectLPFilter(int sensor, int cutoffFreq, BOOL fApply);
BOOL platformHasMag();
char *getBuildInfo();
int  platformGetSysRange();
BOOL platformUseGPS(void);
BOOL platformIsGpsPPSUsed(void);
void platformEnableGpsPps(BOOL enable);
int  platformGetUnitCommunicationType(void);
BOOL platformSetUnitCommunicationType(int type);
void setGpsBaudRate(int16_t rate); 
void setGpsProtocol(int16_t p);   
void platformEnableMag(BOOL enable);
extern char *unitVersionString(void);
extern uint32_t unitSerialNumber(void);
extern uint8_t  unitSpiSwVersion(void);
extern char *platformBuildInfo(void);
extern uint32_t getDacqTime();
void   Reset();
void enableMagUsageInAlgorithm(BOOL enable);
void platformSetGpsBaudRate(int baudRate);
int  platformGetGpsBaudRate(void);
int platformGetGpsProtocol(void);
void platformSetGpsProtocol(int protocol);
int platformGetNextGpsBaudRate(int baudRate);
BOOL platformCanThermistorEnabled();
BOOL platformCanBaudRateDetectEnabled();
BOOL platformCanTermResistorEnabled();
void markEEPROMUnlocked();
void markEEPROMLocked();
BOOL readFlash(uint32_t addr, uint8_t *buf, uint16_t len);
void platformInitConfigureUnit(void);
void platformUnassignSerialChannels();
void platformForceUnassignSerialChannels();
BOOL platformAssignPortTypeToSerialChannel(int portType, int channel);
BOOL platformForcePortTypeToSerialChannel(int portType, int channel);
int  platformGetSerialChannel(int portType);
void platformRegisterRxSerialSemaphoreID(int portType, void *semId);
void platformGetVersionBytes(uint8_t *bytes);
void platformSetCanPacketRate(int rate);
BOOL platformApplyOrientation(uint16_t input);
int  platformGetSensToPpsDelay();
int  platformGetPpsToDrdyDelay();
uint64_t   platformGetCurrTimeStamp();
uint64_t   platformGetCurrTimeStampFromIsr();
uint64_t   platformGetDacqTimeStamp();
void       platformSetDacqTimeStamp(uint64_t time);
uint8_t    platformGetPpsFlag( void );
void       platformSetPpsFlag( uint8_t gotPpsFlag );
uint64_t   platformGetPpsTimeStamp();
void       platformForceMagUsage();
void       platformUpdateDebugPortAssignment();
BOOL       platformMagFilterEnabled();
void       platformEnableMagFilter(BOOL enable);
void       platformUpdateRateFilterType(uint8_t type);
void       platformUpdateAccelFilterType(uint8_t type);
int        platformGetPreFilterType();
int        platformGetFilterCounts(uint32_t type);
int        platformGetFilterType(int sensor);
uint32_t   platformGetIMUCounter();
void       platformUpdateITOW(uint32_t itow);
uint64_t   platformGetEstimatedITOW();
void       platformDetectUserSerialCmd(uint8_t input);
uint64_t   platformGetSolutionTstamp();




/** ***************************************************************************
 * @name performSelfTest() Run self test
 * @brief verify existence of each sensor.
 * ">>swtest"  - data = 0x00
 *
 * @param [in] N/A
 * @retval N/A
 ******************************************************************************/
void performSelfTest();

// sets sensor error state for aalgorithm
extern void    SetSensorError(int sensor);
#define   kick_dog()

#define NUM_UART_PORTS   3

#define USER_SERIAL_PORT   0
#define GPS_SERIAL_PORT    1
#define DEBUG_SERIAL_PORT  2

#define UART_CHANNEL_0     0  // pins 3 and 4 on the 20-pin connector. NOTE: not available in SPI interface mode
#define UART_CHANNEL_1     1  // pins 5 and 6 on the 20-pin connector. NOTE: not available in SPI interface mode 
#define UART_CHANNEL_2     2  // pins 17 and 19 on the 20-pin connector. Always available


#define UART_CHANNEL_NONE -1  // undefined channel

extern int userSerialChan;
extern int gpsSerialChan;
extern int debugSerialChan;

#define LEGACY_BOOT_SEQUENCE 0x0055554A4200A0CELL
#define LEGACY_PING_SEQUENCE 0x005555504B009EF4LL

#endif
