/** ***************************************************************************
 * @file s_eeprom.h legacy functions from older systems that used eeprom
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
#ifndef _EEPROM_API_H
#define _EEPROM_API_H
#include "GlobalConstants.h"

extern void EEPROM_ReadWords(uint16_t addr, uint16_t num, void *destination) ;
extern BOOL EEPROM_WriteWords(uint16_t addr, uint16_t num, void *source) ;
extern void EEPROM_ReadByte(uint16_t addr, uint16_t num, void *destination) ;
extern BOOL EEPROM_WriteByte(uint16_t addr, uint16_t num, void *source) ;
extern void EEPROM_ReadSerialNumber(void *destination) ;
extern void EEPROM_ReadProdConfig(void *destination) ;
extern void EEPROM_ReadCalOffsetAndLength(uint16_t *offset, uint16_t *length);
extern void EEPROM_ReadCalibration(void* destination);
extern void EEPROM_ReadConfiguration(void* destination);
extern void EEPROM_ReadDefaultCalibration(void* destination);
extern void EEPROM_ReadDefaultConfiguration(void* destination);
extern void EEPROM_LockFactoryConfigSector(void);
extern void EEPROM_UnlockFactoryConfigSector(void);
extern BOOL EEPROM_IsConfigSectorLocked(void);
extern BOOL EEPROM_IsErased(void);
extern BOOL EEPROM_WriteDefaultSettings(void);
extern BOOL EEPROM_LoadUserConfig(uint8_t *ptrToUserConfigInRam, int *userConfigSize);
extern BOOL EEPROM_ValidateUserConfig(int *userConfigSize);
extern BOOL EEPROM_EraseUserConfig(void);
extern BOOL EEPROM_SaveUserConfig(uint8_t *ptrToUserConfigStruct, int userConfigStructLen);
extern BOOL EEPROM_IsUserApplicationActive(void);
extern BOOL EEPROM_IsAppStartedFirstTime(void);
extern BOOL EEPROM_SaveUnitConfigurationWords(uint16_t addr, uint16_t numWords, void  *source);
extern BOOL EEPROM_PrepareToEnterBootloader(void);

extern void lockFlash(void);
extern BOOL setJumpFlag(uint32_t dat);


#endif /* S_EEPROM_H */ 


