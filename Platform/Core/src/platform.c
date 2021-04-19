/** ***************************************************************************
 * @file platform.c Initialization for UCB's Comm. and Cal.
 * @Author dan
 * @date   2011-02-09 22:02:39 -0800 (Wed, 09 Feb 2011)
 * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
 * @rev 17479
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 * @version
 * DKH 10.02.14 set sensor range based on EEPROM config
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

#include <string.h>
#include <stdint.h>

#include "config_fields.h"
#include "platformAPI.h"
#include "configuration.h"
#include "boardAPI.h"
#ifdef GPS
#include "gpsAPI.h"
#include "driverGPS.h"
#include "GpsData.h"
#endif
#include "BITStatus.h"
#include "uart.h"
#include "ucb_packet.h"
#include "eepromAPI.h"
#include "lowpass_filter.h"
#include "filter.h"
#include "Indices.h"

#define PACKET_RATE_DIVIDER     10
#define PACKET_CODE             0x4631
#define BAUD_RATE_USER          BAUD_57600  // switches to 38400


ConfigurationStruct gConfiguration;
int                 _interfaceConfigDone = 0;
static int          _communicationType = UART_COMM;
static int          _uartChannel[NUM_UART_PORTS] = {UART_CHANNEL_0, UART_CHANNEL_1, UART_CHANNEL_2}; 

// placeholders for Nav_view compatibility
softwareVersionStruct dupFMversion;  /// 525 digital processor DUP code base
softwareVersionStruct ioupFMversion; /// 525 input output processor IOUP code base
softwareVersionStruct bootFMversion; /// bootloader code base

extern int calibrateTableValid;


#if 0
/** ****************************************************************************
 * @name: initAlgStruct Initialize the algorithm structure NOT CALLED
 * TRACE:
 *      [SDD_INIT_ALG_STRUCT <-- SRC_INIT_ALG_STRUCT]
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void initAlgStruct(void) {

    memset(&gAlgorithm, 0, sizeof(AlgorithmStruct));
    gMagAlign.state = MAG_ALIGN_STATUS_IDLE;
}
#endif


static void _CheckIfUnitHasConfigAndCal(void)
{
    BOOL res = EEPROM_IsErased();
    if(res){
        EEPROM_WriteDefaultSettings();
    }
}





/** ****************************************************************************
 * @name CheckPortBaudRate
 * @brief all serial port baud rates must be 9600, 19200, 38400 or 57600
 * Trace:
 * @param [in] portBaudRate - baud rate
 * @retval 	boolean, TRUE if the packet output is possible, FALSE if not
 ******************************************************************************/
BOOL CheckPortBaudRate (uint16_t portBaudRate)
{
    BOOL valid = TRUE;

    if (portBaudRate >= NUM_BAUD_RATES) {
        valid = FALSE;
    }
    return valid;
}

/** ****************************************************************************
 * @name CheckContPacketRate
 * @brief verify the packet can be 'comfortably' output at the baud rate and
 *        divider rate
 * Trace:
 * [SDD_PKT_CONT_RATE_CHK <-- SRC_PKT_CONT_RATE_CHK]
 * @param [in] outputPacket packet type,
 * @param [in] baud rate
 * @param [in] packetRateDivider - divider
 * @retval 	boolean, TRUE if the packet output is possible, FALSE if not
 ******************************************************************************/
BOOL CheckContPacketRate (uint16_t      outputPacket,
                          uint16_t      baudRate,
                          uint16_t      packetRateDivider)
{
    BOOL     valid = TRUE;
    uint16_t bytesPerPacket;
    uint16_t bytesPerSecond;

    if (packetRateDivider == 0) {
        valid = TRUE;
    } else {
        bytesPerPacket = UCB_SYNC_LENGTH +
                         UCB_PACKET_TYPE_LENGTH +
                         UCB_PAYLOAD_LENGTH_LENGTH +
                         UCB_CRC_LENGTH;

        switch (outputPacket) {
            case UCB_IDENTIFICATION:
                bytesPerPacket += UCB_IDENTIFICATION_LENGTH;
                break;
            case UCB_TEST_0:
                bytesPerPacket += UCB_TEST_0_LENGTH;
                break;
            case UCB_TEST_1:
                bytesPerPacket += UCB_TEST_1_LENGTH;
                break;
            case UCB_FACTORY_1:
                bytesPerPacket += UCB_FACTORY_1_LENGTH;
                break;
            case UCB_FACTORY_2:
                bytesPerPacket += UCB_FACTORY_2_LENGTH;
                break;
            case UCB_VERSION_DATA:
                bytesPerPacket += UCB_VERSION_DATA_LENGTH;
                break;
            case UCB_VERSION_ALL_DATA:
                bytesPerPacket += UCB_VERSION_ALL_DATA_LENGTH;
                break;
            case UCB_SCALED_0:
                bytesPerPacket += UCB_SCALED_0_LENGTH;
                break;
            case UCB_SCALED_1:
                bytesPerPacket += UCB_SCALED_1_LENGTH;
                break;
            default:
                valid = FALSE;
        }
#ifndef USER_PACKETS_NOT_SUPPORTED
        if(outputPacket == UCB_USER_OUT){
            bytesPerPacket += getUserPayloadLength();
            valid = TRUE;
        }
#endif

        bytesPerSecond = bytesPerPacket * (DACQ_200_HZ / platformConvertPacketRateDivider(packetRateDivider));

        //   For a message with 10 bits/byte (data, start, and stop-bits) and a
        //   safety-factor of 80%, determine if the baud-rate can support the
        //   message and output data rate (ODR)
        real dutyCycle = 0.81;
        switch (baudRate) {
            case BAUD_4800:
                valid = (BOOL)(bytesPerSecond < dutyCycle * 480.0 );
                break;
            case BAUD_9600:
                valid = (BOOL)(bytesPerSecond < dutyCycle * 960.0 );
                break;
            case BAUD_19200:
                valid = (BOOL)(bytesPerSecond < dutyCycle * 1920.0 );
                break;
            case BAUD_38400:
                valid = (BOOL)(bytesPerSecond < dutyCycle * 3840.0 );
                break;
            case BAUD_57600:
                valid = (BOOL)(bytesPerSecond < dutyCycle * 5760.0 );
                break;
            case BAUD_115200:
                valid = (BOOL)(bytesPerSecond < dutyCycle * 11520.0 );
                break;
            case BAUD_230400:
                valid = (BOOL)(bytesPerSecond < dutyCycle * 23040.0 );
                break;
            case BAUD_460800:
                valid = (BOOL)(bytesPerSecond < dutyCycle * 46080.0 );
                break;
            default:
                valid = FALSE;
        }
    }
    return valid;
} /* end CheckContPacketRate */


/** ****************************************************************************
 * @name ValidPortConfiguration
 * @brief Check output packet configuration members for sanity
 * Trace: [SDD_PORT_CFG_VALID_01 <-- SRC_PORT_CFG_VALID]
 *        [SDD_PORT_CFG_VALID_02 <-- SRC_PORT_CFG_VALID]
 *        [SDD_PORT_CFG_VALID_03 <-- SRC_PORT_CFG_VALID]
 *        [SDD_PORT_CFG_VALID_04 <-- SRC_PORT_CFG_VALID]
 *        [SDD_PORT_CFG_VALID_05 <-- SRC_PORT_CFG_VALID]
 * @param [in] proposedConfiguration - configuration
 * @retval 	boolean, TRUE if the packet output is possible, FALSE if not
 ******************************************************************************/
BOOL ValidPortConfiguration (ConfigurationStruct *proposedConfiguration)
{
    /// working packet type byte buffer
    uint8_t       type [UCB_PACKET_TYPE_LENGTH];
    UcbPacketType continuousPacketType;
    BOOL          valid = TRUE;

    /// check packet rate divider
    valid &= (BOOL)(CheckPacketRateDivider(proposedConfiguration->packetRateDivider));

    /// get enum for requested continuous packet type
    type[0] = (uint8_t)((proposedConfiguration->packetCode >> 8) & 0xff);
    type[1] = (uint8_t)(proposedConfiguration->packetCode & 0xff);

    continuousPacketType = UcbPacketBytesToPacketType(type);

    /// check that a valid continuous output packet has been selected
    valid &= UcbPacketIsAnOutputPacket(continuousPacketType);

    /// check continuous packet rate
    valid &= CheckContPacketRate( continuousPacketType,
                                  proposedConfiguration->baudRateUser,
                                  proposedConfiguration->packetRateDivider );
    /// check port baud rates
    valid &= CheckPortBaudRate(proposedConfiguration->baudRateUser);

    return valid;
}
/* end ValidPortConfiguration */


/** ****************************************************************************
 * @name DefaultPortConfiguration
 * @brief Set initial ports:
 * port 1 primary UCB, 57600 baud, AU packet, /1 packet divider
 * port 2 CRM input, 38400 baud
 * port 3 9600 baud, unassigned
 * port 4 9600 baud, unassigned
 * defined in extern_port_config.h
 * Trace:
 * [SDD_CFG_PORT_DEF_01 <-- SRC_CFG_PORT_DEF]
 * [SDD_CFG_PORT_DEF_02 <-- SRC_CFG_PORT_DEF]
 *
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void DefaultPortConfiguration (void)
{
  gConfiguration.packetRateDivider = PACKET_RATE_DIVIDER;
  gConfiguration.packetCode        = PACKET_CODE;
  gConfiguration.baudRateUser      = BAUD_RATE_USER;
} /* end DefaultPortConfiguration */


/** ****************************************************************************
 * @name initConfigureUnit initializes the data structures and configurations
 *     that are read from EEPROM for DUP software to run on ADAHRS platform
 * TRACE: * [SDD_INIT_CONFIGURATION_ADAHRS <-- SRC_INIT_CONFIGURE_UNIT]
 * [SDD_EEPROM_INIT_READ <-- SRC_INIT_CONFIGURE_UNIT]
 * [SDD_INIT_MISALIGN_ADAHRS <-- SRC_INIT_CONFIGURE_UNIT]
 * [SDD_EEPROM_CRC_DATA <-- SRC_INIT_CONFIGURE_UNIT]
 * [SDD_INIT_EEPROM_CRC_ADAHRS <-- SRC_INIT_CONFIGURE_UNIT]
 * [SDD_BIT_LIMITS_EEPROM <-- SRC_INIT_CONFIGURE_UNIT]
 *
 * [SDD_CAL_G_DATA_CHECK <-- SRC_INIT_CONFIGURE_UNIT]
 * [SDD_INIT_CONFIGURATION_ORIENT_VALID <-- SRC_INIT_CONFIGURE_UNIT]
 * [SDD_CFG_PORT_DEF_01 <-- SRC_INIT_CONFIGURE_UNIT]
 * [SDD_CFG_PORT_DEF_02 <-- SRC_INIT_CONFIGURE_UNIT]
 * [SDD_INIT_CONFIGURATION_DEFAULT_BARO <-- SRC_INIT_CONFIGURE_UNIT]
 *
 * [SDD_INIT_RPY_OFFSETS_EXTEND <-- SRC_INIT_CONFIGURE_UNIT]
 * [SDD_INIT_RPY_OFFSETS <-- SRC_INIT_CONFIGURE_UNIT]
 * [SDD_INIT_EXT_MAG_CONFIG <-- SRC_INIT_CONFIGURE_UNIT]
 * [SDD_INIT_DUP_SW_VERSION_ADAHRS <-- SRC_INIT_CONFIGURE_UNIT]
 * [SDD_INIT_BOOTLOADER_SW_VERSION <-- SRC_INIT_CONFIGURE_UNIT]
 * [SDD_WATCHDOG <-- SRC_INIT_CONFIGURE_UNIT]
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void platformInitConfigureUnit(void)
{

    lockFlash();

    userSerialChan   = _uartChannel[USER_SERIAL_PORT];
#ifdef GPS
    gpsChan    = _uartChannel[GPS_SERIAL_PORT];
#endif
    debugSerialChan  = _uartChannel[DEBUG_SERIAL_PORT];
    _interfaceConfigDone = 1;

#ifdef GPS
    initGPSDataStruct();
#endif

    _CheckIfUnitHasConfigAndCal();

    EEPROM_ReadConfiguration(&gConfiguration); // s_eeprom.c

    if (gConfiguration.CanBaudRateDetectEnable != true)
      gConfiguration.CanBaudRateDetectEnable = false;
    
    if (gConfiguration.CanTermResistorEnable != true)
      gConfiguration.CanTermResistorEnable = false;
    
    if ((gConfiguration.ecuAddress < 128) || (gConfiguration.ecuAddress > 247))
      gConfiguration.ecuAddress = 128;

    if (gConfiguration.packetRateDivider > 200)
      gConfiguration.packetRateDivider = 10;     // 20 Hz at 200, will show 10 Hz     

    if (gConfiguration.CanOdr > 100)
        gConfiguration.CanOdr = 1;       // 100 Hz

    if (gConfiguration.canPacketType > 0xFF)
        gConfiguration.canPacketType = 0x07;  // ARI, ACCS, SSI2

    /// check user orientation field for validity
    if (CheckOrientation(gConfiguration.orientation.all) == FALSE) {
        gConfiguration.orientation.all = 0; // +x +y +z
    }

    /// check port configuration fields against rules
	// xbow_fields.c
    if (ValidPortConfiguration(&gConfiguration) == FALSE) {
        DefaultPortConfiguration();
    }

    
    if(eepromLocked()){
        gConfiguration.packetRateDivider = 10;
#ifdef CAN_BUS_COMM
        gConfiguration.packetRateDivider = 0;   // quiet mode
#endif
    }
	

    dupFMversion.major = VERSION_MAJOR;
    dupFMversion.minor = VERSION_MINOR;
    dupFMversion.patch = VERSION_PATCH;
    dupFMversion.stage = VERSION_STAGE;
    dupFMversion.build = VERSION_BUILD;



    kick_dog();

}


void readUnitSerialNumber(uint32_t *serialNumber)
{
    EEPROM_ReadSerialNumber((void *)serialNumber);
}

void readProductConfigurationField(void *destination)
{
    EEPROM_ReadProdConfig(destination);
}

void readUnitConfigurationField(uint16_t addr,  uint16_t num, void     *destination)
{
     EEPROM_ReadByte(addr, num, destination);
}

void readUnitConfigurationMemory(uint16_t addr, uint16_t num, void     *destination)
{
     EEPROM_ReadByte(addr, num, destination);
}

void readUnitConfigurationStruct(void* destination)
{
    EEPROM_ReadConfiguration(destination);
}

BOOL saveUnitConfigurationStruct(uint16_t *source)
{
     source++; ///< get past CRC at top

    /// write entire proposed configuration back to EEPROM
    return EEPROM_WriteByte(LOWER_CONFIG_ADDR_BOUND, // 0x1
                           NUM_CONFIG_FIELDS *      // 0x2b - 1 - 2 = 0x28 = 40 byts
                           SIZEOF_WORD,             // 2 bytes
                           (void*)source);

}

/*
BOOL unitHasMag()
{
    return gCalibration.productConfiguration.bit.hasMags;
}
*/


char *platformBuildInfo()
{
    return SOFTWARE_PART;
}


#ifdef	SPI_BUS_COMM

uint8_t unitSpiSwVersion(void)
{
    return SPI_SW_VERSION;
}

#endif


BOOL eepromLocked(void)
{
    if(EEPROM_IsConfigSectorLocked()){
        return TRUE;
    }
    return FALSE;
}

BOOL lockEeprom(void)
{
    if(!EEPROM_IsConfigSectorLocked()){
        EEPROM_LockFactoryConfigSector();
        if(!EEPROM_IsConfigSectorLocked()){
            return FALSE;
        }
    }
    gBitStatus.hwStatus.bit.unlockedEEPROM = FALSE;
    return TRUE;
}

BOOL unlockEeprom(void)
{
     EEPROM_UnlockFactoryConfigSector();

     if(EEPROM_IsConfigSectorLocked()){
         return FALSE;
     }
    
    gBitStatus.hwStatus.bit.unlockedEEPROM = TRUE;
    return TRUE;
}

// returns 0 if OK
BOOL saveEcuAddress(uint16_t *address)
{
    BOOL result = TRUE; // as if error 
     EEPROM_UnlockFactoryConfigSector();

     if(EEPROM_IsConfigSectorLocked()){
         return result;
     }

    result = EEPROM_WriteByte(ECU_ADDRESS_FIELD_ID, 2, address);

    EEPROM_LockFactoryConfigSector();
    
    return result;
}

void markConfigAsInvalid(void)
{
     gBitStatus.swDataBIT.bit.calibrationCRCError = TRUE;
} 



BOOL platformSetPacketRate(int rate, BOOL fApply)
{
    BOOL res;
    uint16_t divider;

    switch(rate){
        case 200:
            divider = PACKET_RATE_DIV_200HZ;
            break;
        case 100:
            divider = PACKET_RATE_DIV_100HZ;
            break;
        case  50:
            divider = PACKET_RATE_DIV_50HZ;
            break;
        case 25:
            divider = PACKET_RATE_DIV_25HZ;
            break;
        case 20:
             divider = PACKET_RATE_DIV_20HZ;
            break;
        case 10:
            divider = PACKET_RATE_DIV_10HZ;
            break;
        case 5:
            divider = PACKET_RATE_DIV_5HZ;
            break;
        case 2:
            divider = PACKET_RATE_DIV_2HZ;
            break;
        case 0:
            divider = PACKET_RATE_DIV_QUIET;
            break;
        default:
            return FALSE;

    }
    
    uint16_t tmp = gConfiguration.packetRateDivider;

    gConfiguration.packetRateDivider =  divider;      // validation uses 100Hz based criteria

    res = ValidPortConfiguration(&gConfiguration); 

    if (res == FALSE || !fApply) {
        gConfiguration.packetRateDivider =  tmp;
    }

    return res;
}

int platformConvertPacketRateDivider(int configParam)
{
    
    if(configParam != PACKET_RATE_DIV_200HZ){
        return configParam*2;
    }
    return 1;
}

int platformGetPacketRateDivider()
{
    
    return gConfiguration.packetRateDivider;
}

void platformSetPacketRateDivider(int div)
{
    gConfiguration.packetRateDivider = div;
}



BOOL platformSetBaudRate(int baudRate, BOOL fApply)
{
    BOOL res;

    switch (baudRate){
        case 9600:	   baudRate = BAUD_9600;  break;
        case 19200:	   baudRate = BAUD_19200;  break;
        case 38400:	   baudRate = BAUD_38400;  break;
        case 57600:	   baudRate = BAUD_57600;  break;
        case 4800:	   baudRate = BAUD_4800;   break;
        case 115200:   baudRate = BAUD_115200; break;
        case 230400:   baudRate = BAUD_230400; break;
        case 460800:   baudRate = BAUD_460800; break;
        default:
            return FALSE;
    }
    uint16_t tmp = gConfiguration.baudRateUser;
    gConfiguration.baudRateUser =  baudRate;
    res = ValidPortConfiguration(&gConfiguration);

    if (res == FALSE || !fApply) {
        gConfiguration.baudRateUser = tmp;
    }

    return res;

}

//#pragma GCC push_options
//#pragma GCC optimize ("O0")
BOOL platformSetOrientation(uint16_t *input, BOOL fApply)
{
    BOOL res;
    uint32_t  orientation = 0;
    uint16_t  sel;
    volatile  int i,j;

    for(i = FORWARD; i <= DOWN; i++){
        sel = input[i];
        j = i;
        switch(sel){
            case PLUS_X:
                switch(j){
                    case FORWARD:
                        orientation |= FWD_X_PLUS_MASK;
                        break;  
                    case RIGHT:
                        orientation |= RIGHT_X_PLUS_MASK;
                        break;
                    case DOWN:
                        orientation |= DOWN_X_PLUS_MASK;
                        break;
                    default:   
                        return FALSE;
                }
                break;
            case PLUS_Y:
                switch(j){
                    case FORWARD:
                        orientation |= FWD_Y_PLUS_MASK;
                        break; 
                    case RIGHT:
                        orientation |= RIGHT_Y_PLUS_MASK;
                        break;
                    case DOWN:
                        orientation |= DOWN_Y_PLUS_MASK;;
                        break;
                    default:
                        return FALSE;
                }
                break;
            case PLUS_Z:
                switch(j){
                    case FORWARD:
                        orientation |= FWD_Z_PLUS_MASK;
                        break; 
                    case RIGHT:
                        orientation |= RIGHT_Z_PLUS_MASK;
                        break;
                    case DOWN:
                        orientation |= DOWN_Z_PLUS_MASK;
                        break;
                    default:
                        return FALSE;
                }
                break;
            case MINUS_X:
                switch(j){
                    case FORWARD:
                        orientation |= FWD_X_MINUS_MASK;
                        break;  
                    case RIGHT:
                        orientation |= RIGHT_X_MINUS_MASK;
                        break;
                    case DOWN:
                        orientation |= DOWN_X_MINUS_MASK;
                        break;
                    default:
                        return FALSE;
                }
                break;
            case MINUS_Y:
                switch(j){
                    case FORWARD:
                        orientation |= FWD_Y_MINUS_MASK;
                        break; 
                    case RIGHT:
                        orientation |= RIGHT_Y_MINUS_MASK;
                        break;
                    case DOWN:
                        orientation |= DOWN_Y_MINUS_MASK;
                        break;
                    default:
                        return FALSE;
                }
                break;
           case MINUS_Z:
                switch(j){
                    case FORWARD:
                        orientation |= FWD_Z_MINUS_MASK;
                        break; 
                    case RIGHT:
                        orientation |= RIGHT_Z_MINUS_MASK;
                        break;
                    case DOWN:
                        orientation |= DOWN_Z_MINUS_MASK;
                        break;
                    default:
                        return FALSE;
                }
                break;
            default:
                return FALSE;
        }

    }

    res = CheckOrientation(orientation);

    if (res == FALSE || !fApply) {
        return res;
    }

    gConfiguration.orientation.all =  orientation;
    *input                         = orientation; 
    return TRUE;

}

BOOL platformApplyOrientation(uint16_t input)
{
    BOOL res = CheckOrientation(input);
    if (res == FALSE) {
        return res;
    }
    gConfiguration.orientation.all =  input;
    return TRUE;
}

uint16_t    platformGetOrientationWord()
{
    return gConfiguration.orientation.all;
}




//#pragma GCC pop_options

int platformGetBaudRate(uint16_t baudCode)
{
    int baudRate = 0;
    if(baudCode == 0xFFFF){
        baudCode = gConfiguration.baudRateUser;
    }
    
    switch (baudCode){
        case BAUD_9600:  baudRate = 9600;  break;
        case BAUD_19200: baudRate = 19200;  break;
        case BAUD_38400: baudRate = 38400;  break;
        case BAUD_4800:	 baudRate = 4800;   break;
        case BAUD_115200: baudRate = 115200; break;
        case BAUD_230400: baudRate = 230400; break;
        case BAUD_460800: baudRate = 460800; break;
        case BAUD_57600:
        default:
            baudRate = 57600;
            break;
    }

    return baudRate;
}

int platformGetPacketRate(uint16_t divider)
{
    int rate = 0;

    if(divider == 0xFFFF){
        divider = gConfiguration.packetRateDivider;
    }

    switch(divider){
        case PACKET_RATE_DIV_200HZ:
            rate = 200;
            break;
        case PACKET_RATE_DIV_100HZ:
            rate = 100;
            break;
        case  PACKET_RATE_DIV_50HZ:
            rate = 50;
            break;
        case PACKET_RATE_DIV_25HZ:
            rate = 25;
            break;
        case PACKET_RATE_DIV_20HZ:
             rate = 20;
            break;
        case PACKET_RATE_DIV_10HZ:
            rate = 10;
            break;
        case PACKET_RATE_DIV_5HZ:
            rate = 5;
            break;
        case PACKET_RATE_DIV_2HZ:
            rate = 2;
            break;
        case 0:
        default:
            break;

    }

    return rate;
}


BOOL  platformSetOutputPacketCode(uint16_t code, BOOL fApply)
{
    uint16_t tmp =  gConfiguration.packetCode;
    BOOL res;

    gConfiguration.packetCode = code;
    res = ValidPortConfiguration(&gConfiguration);

    if (res == FALSE || !fApply) {
        gConfiguration.packetCode =  tmp;
    }

    return res;
}




int platformGetSysRange()
{
    return _400_DPS_RANGE;
}


static BOOL _useGpsPps = FALSE;

BOOL platformIsGpsPPSUsed(void)
{
    return _useGpsPps;
}

void platformEnableGpsPps(BOOL enable)
{
    _useGpsPps = enable;
}


int platformGetGpsBaudRate()
{
    int baudRate = 0;
    
    switch (gConfiguration.baudRateGPS){
        case BAUD_4800:  baudRate = 4800;  break;
        case BAUD_9600:  baudRate = 9600;  break;
        case BAUD_19200: baudRate = 19200;  break;
        case BAUD_38400: baudRate = 38400;  break;
        case BAUD_57600: baudRate = 57600;  break;
        case BAUD_115200: baudRate = 115200; break;
        case BAUD_230400: baudRate = 230400; break;
        case BAUD_460800: baudRate = 460800; break;
        default:
            baudRate = -1;
            break;
    }

    return baudRate;
}

void platformSetGpsBaudRate(int baudRate)
{

    switch (baudRate){
        case 4800:	   baudRate = BAUD_4800;   break;
        case 9600:	   baudRate = BAUD_9600;   break;
        case 19200:	   baudRate = BAUD_19200;  break;
        case 38400:	   baudRate = BAUD_38400;  break;
        case 57600:	   baudRate = BAUD_57600;  break;
        case 115200:   baudRate = BAUD_115200; break;
        case 230400:   baudRate = BAUD_230400; break;
        case 460800:   baudRate = BAUD_460800; break;
        default:
            baudRate = -1;
        break;
    }
    gConfiguration.baudRateGPS =  baudRate;
}

int platformGetGpsProtocol()
{
    return gConfiguration.protocolGPS;
}

void platformSetGpsProtocol(int protocol)
{
    switch(protocol){

     case AUTODETECT:
	 case UBLOX_BINARY:
	 case NOVATEL_BINARY:
	 case NOVATEL_ASCII:
	 case NMEA_TEXT:            // DEFAULT_SEARCH_PROTOCOL
	 case SIRF_BINARY:          // INIT_SEARCH_PROTOCOL
        gConfiguration.protocolGPS = protocol;
        break;
     default:
        gConfiguration.protocolGPS = UNKNOWN;
        break;
    }
}

int platformGetNextGpsBaudRate(int baudRate)
{
    
    switch (baudRate){
        case 4800:   baudRate = 9600;  break;
        case 9600:   baudRate = 19200;  break;
        case 19200:  baudRate = 38400;  break;
        case 38400:  baudRate = 57600;  break;
        case 57600:  baudRate = 1115200;  break;
        case 115200: baudRate = 230400; break;
        case 230400: baudRate = 4800;   break;
        default:
            baudRate = 57600;
            break;
    }

    return baudRate;
}


BOOL platformCanThermistorEnabled()
{
    return gConfiguration.CanTermResistorEnable == TRUE;
}

BOOL platformCanBaudRateDetectEnabled()
{
    return gConfiguration.CanBaudRateDetectEnable == TRUE;
}

void markEEPROMLocked()
{
    gBitStatus.hwStatus.bit.unlockedEEPROM = FALSE;
}

void markEEPROMUnlocked()
{
    gBitStatus.hwStatus.bit.unlockedEEPROM = TRUE;
}

// USER_PORT -> channel 0, GPS_PORT-> channel1, DEBUG_PORT ->channel2 

void platformUnassignSerialChannels()
{
    if(_interfaceConfigDone){
        return;
    }

    _uartChannel[USER_SERIAL_PORT]  = UART_CHANNEL_NONE;
    _uartChannel[GPS_SERIAL_PORT]   = UART_CHANNEL_NONE;
    _uartChannel[DEBUG_SERIAL_PORT] = UART_CHANNEL_NONE;
}

BOOL platformAssignPortTypeToSerialChannel(int portType, int channel)
{
    if(_interfaceConfigDone){
        return FALSE;
    }

    if(_communicationType == SPI_COMM && 
        (channel == UART_CHANNEL_0 || channel == UART_CHANNEL_1)){
        return FALSE; 
    }

    if(portType < USER_SERIAL_PORT || portType > DEBUG_SERIAL_PORT){
        return FALSE; 
    }

    if(channel < UART_CHANNEL_NONE || channel > UART_CHANNEL_2){
        return FALSE; 
    }
    
    if(_uartChannel[portType] == UART_CHANNEL_NONE){
        _uartChannel[portType] = channel;
        return TRUE;
    }
    
    return FALSE;
}

int  platformGetSerialChannel(int portType, BOOL fTx)
{
    return _uartChannel[portType];
}



/******************************************************************************
 * @name getUnitCommunicationType. Retrieves communication interface type
 * @retval N/A
 * @param [out]  - communitation interface type
 ******************************************************************************/
int platformGetUnitCommunicationType(void)
{  
    return _communicationType;
}

/******************************************************************************
 * @name setUnitCommunicationType. 
 * @retval [in] - communitation interface type
 * @param [out] - N/A
 ******************************************************************************/
BOOL platformSetUnitCommunicationType(int type)
{  
    if(_interfaceConfigDone){
        return FALSE;
    }

    if(type != SPI_COMM && type != UART_COMM && type != CAN_BUS){
        return FALSE;
    }

    if(type == SPI_COMM){
        if(!getDataReadyPinState() || BoardIsTestMode()){
            // board in UART mode
            return FALSE;
        }
        for(int i = 0; i < NUM_UART_PORTS; i++){
            if(_uartChannel[i] != UART_CHANNEL_2){
                _uartChannel[i] = UART_CHANNEL_NONE;
            }
        }
    } 

    if(type == CAN_BUS){
        for(int i = 0; i < NUM_UART_PORTS; i++){
            if(_uartChannel[i] != UART_CHANNEL_0){
                _uartChannel[i] = UART_CHANNEL_NONE;
            }
        }
    } 
    
    _communicationType = type;

    return TRUE;
   
}



void  platformRegisterRxSerialSemaphoreID(int portType, void *semId)
{
    uart_registerRxSemaphore(portType, semId);
}

static uint64_t dacqTimer = 0;

uint64_t platformGetDacqTimeStamp()
{
    return dacqTimer;
}

void platformSetDacqTimeStamp(uint64_t time)
{
    dacqTimer = time;
}

void platformGetVersionBytes(uint8_t *bytes)
{
  bytes[0] = (uint8_t)VERSION_MAJOR_NUM;
  bytes[1] = (uint8_t)VERSION_MINOR_NUM;
  bytes[2] = (uint8_t)VERSION_PATCH_NUM;
  bytes[3] = (uint8_t)VERSION_STAGE_NUM;
  bytes[4] = (uint8_t)VERSION_BUILD_NUM;
}

void platformUpdateDebugPortAssignment()
{
    if(_uartChannel[DEBUG_SERIAL_PORT] == _uartChannel[USER_SERIAL_PORT] || debugSerialChan == userSerialChan){
        //user communication takes over
        _uartChannel[DEBUG_SERIAL_PORT] = UART_CHANNEL_NONE;
        debugSerialChan                 = UART_CHANNEL_NONE;        
    }
}

BOOL magFilterEnabled = TRUE;

BOOL platformMagFilterEnabled()
{
    return magFilterEnabled;
}

void platformEnableMagFilter(BOOL enable)
{
    magFilterEnabled = enable;
}


// borrowing the AnalogFilterClocks from the configuration to allow
// the filtering to be changed on the fly.
int  platformGetFilterType(int sensor, BOOL fSpi)
{
    int counts;

    if(sensor == ACCEL_SENSOR){
        counts = gConfiguration.analogFilterClocks[1];
    }else if(sensor == RATE_SENSOR){
        counts = gConfiguration.analogFilterClocks[2];
    }else{
        return -1;
    }
    
    if(fSpi){
        return counts;
    }

    if (counts > 18749 ) {
        return IIR_02HZ_LPF;
    } else if ( (counts <= 18749) && (counts > 8034) ) {
        return IIR_05HZ_LPF;
    } else if ( (counts <= 8034) && (counts > 4017) ) {
        return IIR_10HZ_LPF;
    } else if ( (counts <= 4017) && (counts > 2410) ) {
        return IIR_20HZ_LPF;
    } else if ( (counts <= 2410) && (counts > 1740) ) {
        return IIR_25HZ_LPF;
    } else if ( (counts <= 1740) && (counts > 1204) ) {
        return IIR_40HZ_LPF;
    } else if ( (counts <= 1204) && (counts > 0) ) {
        return IIR_50HZ_LPF;
    } else if (counts == 0) {
        return UNFILTERED;
    } else {
        return 0;  // never hit this, just here to remove compiler warning
    }
}


int     platformGetFilterFrequency(int sensor, BOOL fSpi)
{
    int counts;

    if(sensor == ACCEL_SENSOR){
        counts = gConfiguration.analogFilterClocks[1];
    }else if(sensor == RATE_SENSOR){
        counts = gConfiguration.analogFilterClocks[2];
    }else{
        return -1;
    }
    
    if(fSpi){
        return counts;
    }

    if (counts > 18749 ) {
        return 2;
    } else if ( (counts <= 18749) && (counts > 8034) ) {
        return 5;
    } else if ( (counts <= 8034) && (counts > 4017) ) {
        return 10;
    } else if ( (counts <= 4017) && (counts > 2410) ) {
        return 20;
    } else if ( (counts <= 2410) && (counts > 1740) ) {
        return 25;
    } else if ( (counts <= 1740) && (counts > 1204) ) {
        return 40;
    } else if ( (counts <= 1204) && (counts > 0) ) {
        return 50;
    } else if (counts == 0) {
        return 0;
    } else {
        return 0;  // never hit this, just here to remove compiler warning
    }
}

uint16_t    platformGetFilterFrequencyFromCounts(uint16_t counts)
{
    if (counts > 18749 ) {
        return 2;
    } else if ( (counts <= 18749) && (counts > 8034) ) {
        return 5;
    } else if ( (counts <= 8034) && (counts > 4017) ) {
        return 10;
    } else if ( (counts <= 4017) && (counts > 2410) ) {
        return 20;
    } else if ( (counts <= 2410) && (counts > 1740) ) {
        return 25;
    } else if ( (counts <= 1740) && (counts > 1204) ) {
        return 40;
    } else if ( (counts <= 1204) && (counts > 0) ) {
        return 50;
    } else if (counts == 0) {
        return 0;
    } else {
        return 0;  // never hit this, just here to remove compiler warning
    }

}


int   platformGetPreFilterType()
{
    uint32_t counts = gConfiguration.analogFilterClocks[0];

    if (counts > 18749 || counts == 1) {
        return BWF_LOWPASS_3RD_2;
    } else if ( ((counts <= 18749) && (counts > 8034)) || counts == 2) {
        return BWF_LOWPASS_3RD_5;
    } else if ( ((counts <= 8034) && (counts > 4017)) || counts == 3) {
        return BWF_LOWPASS_3RD_10;
    } else if ( ((counts <= 4017) && (counts > 1740)) || counts == 4) {
        return BWF_LOWPASS_3RD_25;
    } else if ( ((counts <= 1740) && (counts > 20)) || counts == 5 ) {
        return BWF_LOWPASS_3RD_50;
    } else if ( counts == 6 ) {
        return BWF_LOWPASS_3RD_100;
    } else if ( counts == 9) {
        return BWF_LOWPASS_4TH_2;
    } else if ( counts == 10) {
        return BWF_LOWPASS_4TH_5;
    } else if ( counts == 11) {
        return BWF_LOWPASS_4TH_10;
    } else if ( counts == 12) {
        return BWF_LOWPASS_4TH_25;
    } else if ( counts == 13) {
        return BWF_LOWPASS_4TH_50;
    } else if ( counts == 14) {
        return BWF_LOWPASS_4TH_100;
    } else if ( counts == 15) {
        return BWF_LOWPASS_AVERAGE;
    }else if (counts == 0) {
        return BWF_LOWPASS_NONE;
    } else {
        return 0;  // never hit this, just here to remove compiler warning
    }
}



int  platformGetFilterCounts(uint32_t type)
{
    switch(type){
        case IIR_02HZ_LPF:
            return 26785;
        case IIR_05HZ_LPF:
            return 10713;
        case IIR_10HZ_LPF:
            return 5356;
        case IIR_20HZ_LPF:
            return 2678;
        case IIR_25HZ_LPF:
            return 2142;
        case IIR_40HZ_LPF:
            return 1338;
        case UNFILTERED:
            return 0;
        case IIR_50HZ_LPF:
        default:
            return 1070;
    }

}

void platformForceUnassignSerialChannels()
{
    _uartChannel[USER_SERIAL_PORT]  = UART_CHANNEL_NONE;
    _uartChannel[GPS_SERIAL_PORT]   = UART_CHANNEL_NONE;
    _uartChannel[DEBUG_SERIAL_PORT] = UART_CHANNEL_NONE;
    userSerialChan  = -1;
    debugSerialChan = -1;
    gpsChan   = -1;
}

BOOL platformForcePortTypeToSerialChannel(int portType, int channel)
{

    if(_communicationType == SPI_COMM && 
        (channel == UART_CHANNEL_0 || channel == UART_CHANNEL_1)){
        return FALSE; 
    }

    if(portType < USER_SERIAL_PORT || portType > DEBUG_SERIAL_PORT){
        return FALSE; 
    }

    if(channel < UART_CHANNEL_NONE || channel > UART_CHANNEL_2){
        return FALSE; 
    }
    
    if(_uartChannel[portType] == UART_CHANNEL_NONE){
        _uartChannel[portType] = channel;
        switch (portType){
            case USER_SERIAL_PORT:
                userSerialChan   = _uartChannel[USER_SERIAL_PORT];
                break;
            case GPS_SERIAL_PORT:
                gpsChan          = _uartChannel[GPS_SERIAL_PORT];
                break;
            case DEBUG_SERIAL_PORT:
                debugSerialChan = _uartChannel[DEBUG_SERIAL_PORT];
                break;
        } 
        return TRUE;
    }

   
    return FALSE;
}


void platformDetectUserSerialCmd(uint8_t input)
{
    static uint64_t inputSequence = 0LL;

    inputSequence <<= 8;
    inputSequence |= input; 
    inputSequence &= 0x00FFFFFFFFFFFFFFLL; 
    if( inputSequence == LEGACY_BOOT_SEQUENCE ||
        inputSequence == LEGACY_PING_SEQUENCE){
        platformForceUnassignSerialChannels();
        platformForcePortTypeToSerialChannel(USER_SERIAL_PORT, UART_CHANNEL_0);   //switch serial channel to user port

        }
}

void platformSetEcuBaudrate(uint16_t rate)
{
    gConfiguration.ecuBaudRate = rate;    
}

void platformSetEcuAddress(uint16_t address)
{
    gConfiguration.ecuAddress = address;    
}

// Getters 
uint8_t    platformGetEcuAddress()
{
    return gConfiguration.ecuAddress;
}

uint8_t    platformGetEcuBaudrate()
{
    return gConfiguration.ecuBaudRate;    
}

uint8_t    platformGetEcuPacketRate()
{
    return gConfiguration.CanOdr;
}

uint8_t    platformGetEcuPacketType()
{
    return gConfiguration.canPacketType;
}

uint16_t   platformGetEcuBehavior()
{
    return gConfiguration.userBehaviour;
}

void platformSetEcuPacketRate(uint16_t rate)
{
    gConfiguration.CanOdr = rate;    
}

void platformSetEcuPacketType(uint16_t type)
{
    gConfiguration.canPacketType = type;    
}
