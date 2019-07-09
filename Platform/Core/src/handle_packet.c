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
#include "serial_port.h"
#include "config_fields.h"
#include "MagAlign.h"
#include "watchdog.h"
#include "platformAPI.h"
#include "magAPI.h"
#include "eepromAPI.h"
#include "osapi.h"
#include "crc16.h"
#include "uart.h"
#include "ucb_packet.h"

#define UNLOCK_KEY_1 0x19623392
#define UNLOCK_KEY_2 0x85422764

static UcbPacketStruct primaryUcbPacket;    /// all other data

BOOL fReset = FALSE;

void Reset(){fReset = TRUE;}
/// current attached magnetometer serial number
//uint8_t currentMagSerialNumber [RMAG_SERIAL_NUMBER_SIZE] = { 0, 0, 0, 0 };

static void _SetNak(ExternPortTypeEnum port, UcbPacketStruct *ptrUcbPacket);

void WriteMagAlignParamsToMemory( ExternPortTypeEnum port,
                                  UcbPacketStruct    *ptrUcbPacket );

/** ****************************************************************************
 * @name _UcbPing
 * @brief Reply to a PING command
 * Trace: [SDD_UCB_PING <-- SRC_UCB_PING]
 * @param [in] port - number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
static void _UcbPing (ExternPortTypeEnum port,
                      UcbPacketStruct    *ptrUcbPacket)
{
	ptrUcbPacket->payloadLength = 0; /// return ping acknowledgement
	HandleUcbTx(port, ptrUcbPacket);
}

/** ****************************************************************************
 * @name _UcbEcho
 * @brief Reply to an ECHO command
 * Trace: [SDD_UCB_ECHO <-- SRC_UCB_ECHO]
 * @param [in] port - number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
static void _UcbEcho (ExternPortTypeEnum port,
                      UcbPacketStruct    *ptrUcbPacket)
{
	HandleUcbTx(port, ptrUcbPacket);
}

/** ****************************************************************************
 * @name _UcbGetPacket
 * @brief Reply with the requested packet if it is an output packet type
 * Trace:
 *	[SDD_UCB_GETPACKET <-- SRC_UCB_GETPACKET]
 *	[SDD_RESP_ERROR <-- SRC_UCB_GETPACKET]
 * @param [in] port - number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
static void _UcbGetPacket (ExternPortTypeEnum port,
                           UcbPacketStruct    *ptrUcbPacket)
{
    UcbPacketType requestedType;

	if (ptrUcbPacket->payloadLength == 2) {
		requestedType = UcbPacketBytesToPacketType(ptrUcbPacket->payload);

		if (UcbPacketIsAnOutputPacket(requestedType) == TRUE) {
			ptrUcbPacket->packetType = requestedType; ///< response packet type
		 	SendUcbPacket(ptrUcbPacket); ///< generic response packet handler
            return;
		} else {
            _SetNak(port, ptrUcbPacket);
		}
	} else {
        _SetNak(port, ptrUcbPacket);
	}
	HandleUcbTx(port, ptrUcbPacket);
}

/** ****************************************************************************
 * @name _UcbSetFields
 * @brief handles a UCB set fields command packet
 * Trace:
 * [SDD_UCB_SETFIELDS <-- SRC_UCB_SETFIELDS]
 * [SDD_UCB_SETFIELDS_ID <-- SRC_UCB_SETFIELDS]
 * [SDD_UCB_SETFIELDS_DATA <-- SRC_UCB_SETFIELDS]
 * [SDD_UCB_SETFIELDS_NAK1 <-- SRC_UCB_SETFIELDS]
 * [SDD_UCB_SETFIELDS_NAK2 <-- SRC_UCB_SETFIELDS]
 * [SDD_UCB_SETFIELDS_PAIR_VALID <-- SRC_UCB_SETFIELDS]
 * @param [in] port - number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
static void _UcbSetFields (ExternPortTypeEnum port,
                           UcbPacketStruct    *ptrUcbPacket)

{
	uint8_t numFields = ptrUcbPacket->payload[0];
	uint8_t fieldCount;
	uint8_t validFieldCount;
    /** some fields need to be set together, so collect all field ID's and data
        in one set of arrays */
	uint16_t fieldId   [UCB_MAX_PAYLOAD_LENGTH / 4];
    /// array sizes are based on maximum number of fields to change
	uint16_t fieldData [UCB_MAX_PAYLOAD_LENGTH / 4];

	/// verify that the packet length matches packet specification
    if ((numFields > 0) &&
    	(ptrUcbPacket->payloadLength == (1 + numFields * 4)))
    {
	    /// loop through all fields and data specified in set fields request
	    for (fieldCount = 0; fieldCount < numFields; ++fieldCount) {
	    	/// read field ID and field data from packet into usable arrays
            fieldId[fieldCount]   = (uint16_t)((ptrUcbPacket->payload[(fieldCount * 4) + 1] << 8) |
                                                ptrUcbPacket->payload[(fieldCount * 4) + 2]);
            fieldData[fieldCount] = (uint16_t)((ptrUcbPacket->payload[(fieldCount * 4) + 3] << 8) |
                                                ptrUcbPacket->payload[(fieldCount * 4) + 4]);
	    }

	    validFieldCount = CheckRamFieldData(numFields, fieldId, fieldData, fieldId);
		if (validFieldCount > 0) {	/// all or some requested field changes valid?
			/// build and send positive acknowledgement packet
			ptrUcbPacket->payloadLength = (uint8_t)(1 + (validFieldCount * 2));
	    	ptrUcbPacket->payload[0]    = validFieldCount; /// number of valid fields

			/// place valid field ID's in payload
			for (fieldCount = 0; fieldCount < validFieldCount; ++fieldCount) {
				ptrUcbPacket->payload[(fieldCount * 2) + 1] = (uint8_t)((fieldId[fieldCount] >> 8) & 0xff);
				ptrUcbPacket->payload[(fieldCount * 2) + 2] = (uint8_t)( fieldId[fieldCount]       & 0xff);
			}
	        HandleUcbTx(port, ptrUcbPacket); ///< send acknowledgement
		}

		/// any invalid requested field changes?
		if (validFieldCount < numFields) {
            _SetNak(port, ptrUcbPacket);
     	    HandleUcbTx(port, ptrUcbPacket);
		}

		if (validFieldCount > 0) { /// apply any changes
			SetFieldData(); // xbowsp_fields.c
		}
	} else {
        _SetNak(port, ptrUcbPacket);
	    HandleUcbTx(port, ptrUcbPacket);
	}
}

/** ****************************************************************************
 * @name _UcbGetFields
 * @brief Handles UCB get fields command packet
 * Trace:
 * [SDD_UCB_GETFIELDS <-- SRC_UCB_GETFIELDS]
 * [SDD_UCB_GETFIELDS_ID <-- SRC_UCB_GETFIELDS]
 * [SDD_UCB_GETFIELDS_NAK1 <-- SRC_UCB_GETFIELDS]
 * [SDD_UCB_GETFIELDS_NAK2 <-- SRC_UCB_GETFIELDS]
 * @param [in] port - number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
static void _UcbGetFields (ExternPortTypeEnum port,
                           UcbPacketStruct    *ptrUcbPacket)
{
	uint8_t  numFields = ptrUcbPacket->payload[0];
	uint8_t  fieldCount;
	uint8_t  validFieldCount = 0;
	uint16_t fieldId [UCB_MAX_PAYLOAD_LENGTH / 4];

	/// verify that the packet length matches packet specification
    if ((numFields > 0) &&
    	(ptrUcbPacket->payloadLength == (1 + numFields * 2))) {

        /// read all fields specified in get fields request
        for (fieldCount = 0; fieldCount < numFields; ++fieldCount) {
            /// read field ID from packet into usable array
            fieldId[validFieldCount] = (uint16_t)((ptrUcbPacket->payload[(fieldCount * 2) + 1] << 8) |
                                                   ptrUcbPacket->payload[(fieldCount * 2) + 2]);

            /// check get field address bounds
            if (((fieldId[validFieldCount] >= LOWER_CONFIG_ADDR_BOUND) &&
                 (fieldId[validFieldCount] <= UPPER_CONFIG_ADDR_BOUND)) ||
                 (fieldId[validFieldCount] == PRODUCT_CONFIGURATION_FIELD_ID)) {
                ++validFieldCount;
            }
        }

        if (validFieldCount > 0) {	/// all or some requested get field addresses valid?
            /// build and return valid get fields with data packet
            ptrUcbPacket->payloadLength = (uint8_t)(1 + validFieldCount * 4);

            /// number of fields being returned
            ptrUcbPacket->payload[0] = validFieldCount;

            /// retrieve all fields specified in get fields request
            for (fieldCount = 0; fieldCount < validFieldCount; ++fieldCount) {
                /** product configuration field is out of normal
                    configuration address range, needs to be fetched from
                    calibration structure */
                if (fieldId[fieldCount] == PRODUCT_CONFIGURATION_FIELD_ID) {
                    ptrUcbPacket->payload[(fieldCount * 4) + 1] = (uint8_t)((fieldId[fieldCount] >> 8) & 0xff);
                    ptrUcbPacket->payload[(fieldCount * 4) + 2] = (uint8_t)( fieldId[fieldCount]       & 0xff);
                    ptrUcbPacket->payload[(fieldCount * 4) + 3] = (uint8_t)((unitProductConfiguration() >> 8) & 0xff);
                    ptrUcbPacket->payload[(fieldCount * 4) + 4] = (uint8_t)( unitProductConfiguration()       & 0xff);
                }
                else {	/// normal field, exists in configuration structure
                    ptrUcbPacket->payload[(fieldCount * 4) + 1] = (uint8_t)((fieldId[fieldCount] >> 8) & 0xff);
                    ptrUcbPacket->payload[(fieldCount * 4) + 2] = (uint8_t)( fieldId[fieldCount]       & 0xff);
                    ptrUcbPacket->payload[(fieldCount * 4) + 3] = (uint8_t)((((uint16_t *)&gConfiguration)[(fieldId[fieldCount])] >> 8) & 0xff);
                    ptrUcbPacket->payload[(fieldCount * 4) + 4] = (uint8_t)( ((uint16_t *)&gConfiguration)[(fieldId[fieldCount])]       & 0xff);
                }
            }
            HandleUcbTx(port, ptrUcbPacket);
        }

        /// any invalid get fields addresses?
        if (validFieldCount < numFields) {
            _SetNak(port, ptrUcbPacket);
            HandleUcbTx(port, ptrUcbPacket);
        }
    } else {
        _SetNak(port, ptrUcbPacket);
        HandleUcbTx(port, ptrUcbPacket);
    }
}

/** ****************************************************************************
 * @name _UcbReadFields
 * @brief Handles UCB read fields command
 * Trace:
 * [SDD_UCB_READFIELDS <-- SRC_UCB_READFIELDS]
 * [SDD_UCB_READFIELDS_ID <-- SRC_UCB_READFIELDS]
 * [SDD_UCB_READFIELDS_NAK1 <-- SRC_UCB_READFIELDS]
 * [SDD_UCB_READFIELDS_NAK2 <-- SRC_UCB_READFIELDS]
 * @param [in] port - number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
static void _UcbReadFields (ExternPortTypeEnum port,
                            UcbPacketStruct    *ptrUcbPacket)
{
	uint8_t  numFields = ptrUcbPacket->payload[0];
	uint8_t  fieldCount;
	uint8_t  validFieldCount = 0;
	uint16_t fieldId [UCB_MAX_PAYLOAD_LENGTH / 4];
	uint16_t fieldData;

    SetMaxDelay_Watchdog(); // Set the watchdog delay to its maximum value

    /// verify that the packet length matches packet specification
    if ((numFields > 0) &&
    (ptrUcbPacket->payloadLength == (1 + numFields * 2))) {
        /// read all fields specified in get fields request
        for (fieldCount = 0; fieldCount < numFields; ++fieldCount) {
            /// read field ID from packet into usable array
            fieldId[validFieldCount] = (uint16_t)((ptrUcbPacket->payload[(fieldCount * 2) + 1] << 8) |
                                                   ptrUcbPacket->payload[(fieldCount * 2) + 2]);

            /// check read field address bounds
            if (((fieldId[validFieldCount] >= LOWER_CONFIG_ADDR_BOUND) &&
                 (fieldId[validFieldCount] <= UPPER_CONFIG_ADDR_BOUND)) ||
                 (fieldId[validFieldCount] == PRODUCT_CONFIGURATION_FIELD_ID)) {
                ++validFieldCount;
            }
        }

        if (validFieldCount > 0) { /// all or some requested addresses valid?
            /// build and return valid get fields with data packet
            ptrUcbPacket->payloadLength = (uint8_t)(1 + validFieldCount * 4);

            ptrUcbPacket->payload[0] = validFieldCount; ///< # being returned

            /// retrieve all fields specified in get fields request
            for (fieldCount = 0; fieldCount < validFieldCount; ++fieldCount) {
                /** product configuration field is out of normal configuration
                    address range, needs to be fetched from calibration
                    structure */
                if (fieldId[fieldCount] == PRODUCT_CONFIGURATION_FIELD_ID) {
                    ptrUcbPacket->payload[(fieldCount * 4) + 1] = (uint8_t)((fieldId[fieldCount] >> 8) & 0xff);
                    ptrUcbPacket->payload[(fieldCount * 4) + 2] = (uint8_t)( fieldId[fieldCount]       & 0xff);

                    readProductConfigurationField( &fieldData);
                    ptrUcbPacket->payload[(fieldCount * 4) + 3] = (uint8_t)((fieldData >> 8) & 0xff);
                    ptrUcbPacket->payload[(fieldCount * 4) + 4] = (uint8_t)( fieldData       & 0xff);
                } else {	/// normal field, exists in configuration structure
                    ptrUcbPacket->payload[(fieldCount * 4) + 1] = (uint8_t)((fieldId[fieldCount] >> 8) & 0xff);
                    ptrUcbPacket->payload[(fieldCount * 4) + 2] = (uint8_t)( fieldId[fieldCount]       & 0xff);
                    /// read field from EEPROM
                    readUnitConfigurationField(fieldId[fieldCount], sizeof(fieldData), &fieldData);
                    ptrUcbPacket->payload[(fieldCount * 4) + 3] = (uint8_t)((fieldData >> 8) & 0xff);
                    ptrUcbPacket->payload[(fieldCount * 4) + 4] = (uint8_t)( fieldData       & 0xff);
                }
            }
            HandleUcbTx(port, ptrUcbPacket);
        }

        /// invalid get fields addresses?
        if (validFieldCount < numFields) {
            _SetNak(port, ptrUcbPacket);
            HandleUcbTx(port, ptrUcbPacket);
        }
    } else {
        _SetNak(port, ptrUcbPacket);
        HandleUcbTx(port, ptrUcbPacket);
    }
    RestoreDelay_Watchdog();
}

/** ****************************************************************************
 * @name _UcbWriteFields
 * @briefHandle UCB write fields command packet
 * Trace:
 * [SDD_UCB_WRITEFIELDS <-- SRC_UCB_WRITEFIELDS]
 * [SDD_UCB_WRITEFIELDS_ID <-- SRC_UCB_WRITEFIELDS]
 * [SDD_UCB_WRITEFIELDS_NAK1 <-- SRC_UCB_WRITEFIELDS]
 * [SDD_UCB_WRITEFIELDS_NAK2 <-- SRC_UCB_WRITEFIELDS]
 * [SDD_UCB_WRITEFIELDS_DATA <-- SRC_UCB_WRITEFIELDS]
 * [SDD_UCB_WRITEFIELDS_PAIR_VALID <-- SRC_UCB_WRITEFIELDS]
 *
 * @param [in] port - number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
static void _UcbWriteFields (ExternPortTypeEnum port,
                             UcbPacketStruct    *ptrUcbPacket)
{
	uint8_t  numFields = ptrUcbPacket->payload[0];
	uint8_t  fieldCount;
	uint8_t  validFieldCount;
    /** some fields need to be set together, so collect all field ID's and data
       in one set of arrays */
	uint16_t fieldId   [UCB_MAX_PAYLOAD_LENGTH / 4];
    /// array sizes are based on maximum number of fields to change
	uint16_t fieldData [UCB_MAX_PAYLOAD_LENGTH / 4];

    SetMaxDelay_Watchdog(); // Set the watchdog delay to its maximum value

    /// verify that the packet length matches packet specification
    if( ( numFields > 0 ) &&
        ( ptrUcbPacket->payloadLength == (1 + numFields * 4) ) && !eepromLocked())
    {
        /// loop through all fields and data specified in set fields request
        for (fieldCount = 0; fieldCount < numFields; ++fieldCount) {
            /// read field ID and field data from packet into usable arrays
            fieldId[fieldCount]   = (uint16_t)((ptrUcbPacket->payload[(fieldCount * 4) + 1] << 8) |
                                                ptrUcbPacket->payload[(fieldCount * 4) + 2]);
            fieldData[fieldCount] = (uint16_t)((ptrUcbPacket->payload[(fieldCount * 4) + 3] << 8) |
                                                ptrUcbPacket->payload[(fieldCount * 4) + 4]);
        }

        /// check if data to set is valid xbowsp_fields.c
        validFieldCount = CheckEepromFieldData(numFields,
                                               fieldId,
                                               fieldData,
                                               fieldId);
// there is no check for corect number of changed fields only that something has changed
        if (validFieldCount > 0) { ///< all or some requested field changes valid?
            /// apply any changes
            if (WriteFieldData() == TRUE) { // xbowsp_fields.c
                /// build and send positive acknowledgement packet
                ptrUcbPacket->payloadLength = (uint8_t)(1 + (validFieldCount * 2));

                /// number of valid fields
                ptrUcbPacket->payload[0] = validFieldCount;

                /// place valid field ID's in payload
                for (fieldCount = 0; fieldCount < validFieldCount; ++fieldCount) {
                    ptrUcbPacket->payload[(fieldCount * 2) + 1] = (uint8_t)((fieldId[fieldCount] >> 8) & 0xff);
                    ptrUcbPacket->payload[(fieldCount * 2) + 2] = (uint8_t)( fieldId[fieldCount]       & 0xff);
                }
                HandleUcbTx(port, ptrUcbPacket);
            } else {
                _SetNak(port, ptrUcbPacket);
                HandleUcbTx(port, ptrUcbPacket);
            }
        }

        /// any invalid requested field changes?
        if (validFieldCount < numFields) {
            _SetNak(port, ptrUcbPacket);
            HandleUcbTx(port, ptrUcbPacket);
        }
    } else {
        _SetNak(port, ptrUcbPacket);
        HandleUcbTx(port, ptrUcbPacket);
    }
    RestoreDelay_Watchdog(); // Restore the watchdog delay to its original value
}


/** ****************************************************************************
 * @name _UcbUnlockEeprom
 * @brief unlock the EEPROM if the CRC of the unit serial number and payload is 0
 * Trace:
 *	[SDD_UCB_UNLOCK_EEPROM <-- SRC_UCB_UNLOCK_EEPROM]
 *
 * @param [in] port -  number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
static void _UcbUnlockEeprom (ExternPortTypeEnum port,
                              UcbPacketStruct    *ptrUcbPacket)
{
	uint16_t crc = 1;
    uint8_t  serialNumberBytes [6];
    uint32_t SN, payloadLen = ptrUcbPacket->payloadLength;
    uint32_t *keyPtr = (uint32_t*)ptrUcbPacket->payload;    
    readUnitSerialNumber(&SN);

    if(payloadLen == 8){
        if(keyPtr[0] == UNLOCK_KEY_1 && keyPtr[1] == UNLOCK_KEY_2){
            crc = 0;
        }
    }else {

	/// low 16-bits of SN first, little endian order
	serialNumberBytes[0] = (uint8_t)(SN & 0xff);
	serialNumberBytes[1] = (uint8_t)((SN >> 8) & 0xff);
	/// high 16-bits of SN next, little endian order
	serialNumberBytes[2] = (uint8_t)((SN >> 16) & 0xff);
	serialNumberBytes[3] = (uint8_t)((SN >> 24) & 0xff);
        /// CRC in payload
	serialNumberBytes[4] = ptrUcbPacket->payload[0];
	serialNumberBytes[5] = ptrUcbPacket->payload[1];
        crc = CalculateCRC(serialNumberBytes, 6);
    }
    
    if (crc == 0)
    { ///< correct unlock code?
        ptrUcbPacket->payloadLength = 0;
        if (eepromLocked())
        {
            if (unlockEeprom() != TRUE)
            {
                _SetNak(port, ptrUcbPacket);
            }
            else
            {
                EEPROM_EraseUserConfig();
                markEEPROMUnlocked();
             }
        }
    }
    else
    {
        _SetNak(port, ptrUcbPacket);
	}
	HandleUcbTx(port, ptrUcbPacket);
} /* end HandleUcbUnlockEeprom() */

/** ****************************************************************************
 * @name _UcbUnlockEeprom
 * @brief lock xbow EEPROM to disable user modification
 * Trace:
 *	[SDD_UCB_LOCK_EEPROM <-- SRC_UCB_LOCK_EEPROM]
 *
 * @param [in] port -  number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
static void _UcbLockEeprom (ExternPortTypeEnum port,
                              UcbPacketStruct    *ptrUcbPacket)
{
    // NEEDS TO BE CHECKED
    if(!lockEeprom()){
        _SetNak(port, ptrUcbPacket);
    }else {
         ptrUcbPacket->payloadLength = 0;
         markEEPROMLocked();
    }
	HandleUcbTx(port, ptrUcbPacket);
} /* end HandleUcbUnlockEeprom() */


/** ****************************************************************************
 * @name _UcbReadEeprom
 * @brief Read 16 bit cells from EEPROM, passed in starting address and number
 * of cells in the packet payload
 *
 * Trace:
 *	[SDD_UCB_READ_EEPROM <-- SRC_UCB_READ_EEPROM]
 *   [SDD_UCB_READ_EEPROM_ERROR <-- SRC_UCB_READ_EEPROM]
 *
 * @param [in] port -  number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
static void _UcbReadEeprom (ExternPortTypeEnum port,
                                 UcbPacketStruct    *ptrUcbPacket)
{
    uint16_t startAddress;
    uint8_t  wordsToRead;
    uint8_t  bytesToRead;

    SetMaxDelay_Watchdog(); ///< Set the watchdog delay to its maximum value

    startAddress = (uint16_t)((ptrUcbPacket->payload[0] << 8) |
                               ptrUcbPacket->payload[1]);
    wordsToRead  = ptrUcbPacket->payload[2];
    bytesToRead  = (uint8_t)(wordsToRead * 2);

    /// verify that the packet length matches packet specification
    if (ptrUcbPacket->payloadLength == 3) {
        ptrUcbPacket->payloadLength = (uint8_t)(ptrUcbPacket->payloadLength + bytesToRead);
        readUnitConfigurationMemory(startAddress,
                       bytesToRead,
                       &(ptrUcbPacket->payload[3]));
	} else {
        _SetNak(port, ptrUcbPacket);
    }
	HandleUcbTx(port, ptrUcbPacket);
    RestoreDelay_Watchdog(); /// Restore the watchdog delay to its original value
}

/** ****************************************************************************
 * @name _UcbWriteEeprom
 * @brief Write data as 16 bit cells into an unlocked EEPROM.
 * Trace:
 *	[SDD_UCB_WRITE_EEPROM <-- SRC_UCB_WRITE_EEPROM]
 *	[SDD_UCB_WRITE_EEPROM_ERROR <-- SRC_UCB_WRITE_EEPROM]
 *
 * @param [in] port -  number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
static void _UcbWriteEeprom (ExternPortTypeEnum port,
                             UcbPacketStruct    *ptrUcbPacket)
{
    uint16_t startAddress;
    uint8_t  wordsToWrite;
    uint16_t bytesToWrite;

    startAddress = (uint16_t)((ptrUcbPacket->payload[0] << 8) |
                               ptrUcbPacket->payload[1]);
    wordsToWrite = ptrUcbPacket->payload[2];
    bytesToWrite = (uint16_t)wordsToWrite * 2;

    SetMaxDelay_Watchdog();

    /// verify that the packet length matches packet specification
    if ((ptrUcbPacket->payloadLength == (bytesToWrite + 3)) && !eepromLocked()) {
        /// flag current CRC as invalid
        markConfigAsInvalid();
        /// 0 means no errors
        if (EEPROM_SaveUnitConfigurationWords(startAddress,
                             wordsToWrite,
                             &(ptrUcbPacket->payload[3])) == 0) {
            ptrUcbPacket->payloadLength = 3;
        } else {
            _SetNak(port, ptrUcbPacket);
        }
    } else {
        _SetNak(port, ptrUcbPacket);
    }

    HandleUcbTx(port, ptrUcbPacket);
    RestoreDelay_Watchdog();
}

/** ****************************************************************************
 * Function name:	_UcbProgramReset
 * @brief force watch dog reset
 * Trace: [SDD_UCB_WD_RESET <-- SRC_UCB_PROG_RESET]
 *
 * @param [in] port -  number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
static void _UcbProgramReset (ExternPortTypeEnum port,
                              UcbPacketStruct    *ptrUcbPacket)
{
	HandleUcbTx(port, ptrUcbPacket);

    OS_Delay(10);

    NVIC_SystemReset();
}

/** ****************************************************************************
 * @name _UcbSoftwareReset
 * @brief Force a watchdog reset from the above function.
 *
 * Trace: [SDD_UCB_SW_RESET <-- SRC_UCB_SW_RESET]
 * softwareReset
 * @param [in] port -  number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
static void _UcbSoftwareReset (ExternPortTypeEnum port,
                                UcbPacketStruct    *ptrUcbPacket)
{

    /// return software reset acknowledgement
	HandleUcbTx(port, ptrUcbPacket);

    OS_Delay(10);

    NVIC_SystemReset();
}


/** ****************************************************************************
 * @name _UcbWriteCal
 * @brief Handles UCB write calibration (Mag align) command packet(s), commands
 * include start, stop and write to eeprom
 *
 * Trace:
 *	[SDD_MAG_CAL_MAG_PRESENT_01 <-- SRC_UCB_WRITE_CAL]
 * [SDD_MAG_CAL_RESET_01 <-- SRC_UCB_WRITE_CAL]
 * [SDD_MAG_CAL_RESET_02 <-- SRC_UCB_WRITE_CAL]
 * [SDD_MAG_CAL_RESET_03 <-- SRC_UCB_WRITE_CAL]
 *  [SDD_MAG_CAL_PHASE_1_01 <-- SRC_UCB_WRITE_CAL]
 * [SDD_MAG_CAL_PHASE_1_02 <-- SRC_UCB_WRITE_CAL]
 * [SDD_MAG_CAL_PHASE_1_03 <-- SRC_UCB_WRITE_CAL]
 * [SDD_MAG_CAL_PHASE_1_04 <-- SRC_UCB_WRITE_CAL]
 * [SDD_MAG_CAL_PHASE_1_COMPLETE_01 <-- SRC_UCB_WRITE_CAL]
 * [SDD_MAG_CAL_PHASE_1_COMPLETE_02 <-- SRC_UCB_WRITE_CAL]
 * [SDD_MAG_CAL_PHASE_1_COMPLETE_03 <-- SRC_UCB_WRITE_CAL]
 * [SDD_MAG_CAL_PHASE_1_COMPLETE_04 <-- SRC_UCB_WRITE_CAL]
 * [SDD_MAG_CAL_PHASE_1_COMPLETE_05 <-- SRC_UCB_WRITE_CAL]
 *
 * [SDD_MAG_CAL_PHASE_2_01 <-- SRC_UCB_WRITE_CAL]
 * [SDD_MAG_CAL_PHASE_2_02 <-- SRC_UCB_WRITE_CAL]
 * [SDD_MAG_CAL_PHASE_2_03 <-- SRC_UCB_WRITE_CAL]
 * [SDD_MAG_CAL_PHASE_2_04 <-- SRC_UCB_WRITE_CAL]
 * [SDD_MAG_CAL_PHASE_2_05 <-- SRC_UCB_WRITE_CAL]
 * [SDD_MAG_CAL_PHASE_2_COMPLETE_01 <-- SRC_UCB_WRITE_CAL]
 * [SDD_MAG_CAL_PHASE_2_COMPLETE_02 <-- SRC_UCB_WRITE_CAL]
 * [SDD_MAG_CAL_PHASE_2_COMPLETE_03 <-- SRC_UCB_WRITE_CAL]
 * [SDD_MAG_CAL_PHASE_2_COMPLETE_04 <-- SRC_UCB_WRITE_CAL]
 * [SDD_MAG_CAL_PHASE_2_COMPLETE_05 <-- SRC_UCB_WRITE_CAL]
 * [SDD_MAG_CAL_PHASE_2_COMPLETE_06 <-- SRC_UCB_WRITE_CAL]
 * [SDD_MAG_CAL_PHASE_2_COMPLETE_07 <-- SRC_UCB_WRITE_CAL]
 * [SDD_MAG_CAL_PHASE_2_TERMINATE_01 <-- SRC_UCB_WRITE_CAL]
 * [SDD_MAG_CAL_PHASE_2_TERMINATE_02 <-- SRC_UCB_WRITE_CAL]
 * [SDD_MAG_CAL_PHASE_2_TERMINATE_03 <-- SRC_UCB_WRITE_CAL]
 * [SDD_MAG_CAL_PHASE_2_TERMINATE_04 <-- SRC_UCB_WRITE_CAL]
 * [SDD_MAG_CAL_PHASE_2_TERMINATE_05 <-- SRC_UCB_WRITE_CAL]
 * [SDD_MAG_CAL_STORE_CAL_01 <-- SRC_UCB_WRITE_CAL]
 * [SDD_MAG_CAL_STORE_CAL_02 <-- SRC_UCB_WRITE_CAL]
 * [SDD_MAG_CAL_STORE_CAL_03 <-- SRC_UCB_WRITE_CAL]
 * [SDD_MAG_CAL_STORE_CAL_04 <-- SRC_UCB_WRITE_CAL]
 * @param [in] port - number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
static void _UcbWriteCal (ExternPortTypeEnum port,
                          UcbPacketStruct    *ptrUcbPacket)
{
    /// tells calibrate routine to terminate phase 2 alignment
    uint16_t   calRequest = (uint16_t)((ptrUcbPacket->payload[0] << 8) |
                                        ptrUcbPacket->payload[1]);

    /** verify that the packet length matches packet specification
	and a magnetometer is present */
    if ((ptrUcbPacket->payloadLength == 2) &&  platformHasMag()) {

        /// handle specific calibration request
        switch (calRequest) {
            case MAG_ALIGN_STATUS_LEVEL_START:
                /// start phase 1 - axis leveling
                SetMagAlignState(MAG_ALIGN_STATUS_LEVEL_START);
                break;
            case MAG_ALIGN_STATUS_START_CAL_WITHOUT_AUTOEND:
                /// start phase 2 - alignment WITHOUT auto termination
                SetMagAlignState(MAG_ALIGN_STATUS_START_CAL_WITHOUT_AUTOEND);
                break;
            case MAG_ALIGN_STATUS_START_CAL_WITH_AUTOEND:
                /// start phase 2 - alignment WITH auto termination
                SetMagAlignState(MAG_ALIGN_STATUS_START_CAL_WITH_AUTOEND);
                break;
            case MAG_ALIGN_STATUS_TERMINATION:
                if ((GetMagAlignState() == MAG_ALIGN_STATUS_START_CAL_WITHOUT_AUTOEND) ||
                    (GetMagAlignState() == MAG_ALIGN_STATUS_START_CAL_WITH_AUTOEND)) {
                        /// terminate phase 2 - alignment
                        SetMagAlignState(MAG_ALIGN_STATUS_IDLE);
                    } else {
                        _SetNak(port, ptrUcbPacket);                }
                break;
            case MAG_ALIGN_STATUS_SAVE2EEPROM:
                if(!eepromLocked()){
                    WriteMagAlignParamsToMemory(port, ptrUcbPacket);
                    break;
                }
                // fall through
            default: /// unrecognized calibration request
                _SetNak(port, ptrUcbPacket);
                break;
        }
    } else {
        _SetNak(port, ptrUcbPacket);
    }
    HandleUcbTx(port, ptrUcbPacket);
}

/** ****************************************************************************
 * @name _SetNak
 * @brief set up UCB error NAK packet. Return NAK with requested packet type in
 *        data field. HandleUcbTx() needs to be called.
 * Trace:
 * @param [in] port - port type UCB or CRM
 * @param [out] packetPtr - filled in packet from the mapped physical port
 * @retval N/A
 ******************************************************************************/
static void _SetNak (ExternPortTypeEnum port,
                     UcbPacketStruct    *ptrUcbPacket)
{
    UcbPacketPacketTypeToBytes(ptrUcbPacket->packetType,
                               ptrUcbPacket->payload);

	/// return NAK, requested packet type placed in data field by external port
	ptrUcbPacket->packetType 	= UCB_NAK;
	ptrUcbPacket->payloadLength = UCB_PACKET_TYPE_LENGTH;
}

/** ****************************************************************************
 * @name _UcbError
 * @brief UCB error packet
 * Trace: [SDD_UCB_UNKNOWN_02 <-- SRC_UCB_UNKNOWN]
 *        [SDD_UCB_TIMEOUT_02 <-- SRC_UCB_TIMEOUT_REPLY]
 *        [SDD_UCB_CRC_FAIL_02 <-- SRC_UCB_CRCFAIL_REPLY]
 * @param [in] port - port type UCB or CRM
 * @param [out] packetPtr - filled in packet from the mapped physical port
 * @retval N/A
 ******************************************************************************/
static void _UcbError (ExternPortTypeEnum port,
                       UcbPacketStruct    *ptrUcbPacket)
{
	/// return NAK, requested packet type placed in data field by external port
	ptrUcbPacket->packetType 	= UCB_NAK;
	ptrUcbPacket->payloadLength = UCB_PACKET_TYPE_LENGTH;
    HandleUcbTx(port, ptrUcbPacket);
}

/** ****************************************************************************
 * @name _UcbJump2IAP
 * @brief unlock the EEPROM if the CRC of the unit serial number and payload is 0
 * Trace:
 *	[SDD_UCB_UNLOCK_EEPROM <-- SRC_UCB_UNLOCK_EEPROM]
 *
 * @param [in] port -  number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
static void _UcbJump2IAP (ExternPortTypeEnum port,
                              UcbPacketStruct    *ptrUcbPacket)
{
	//set need update app flag
    if(EEPROM_PrepareToEnterBootloader())
    {
		_SetNak(port,ptrUcbPacket);
    }
	HandleUcbTx(port, ptrUcbPacket);

    OS_Delay(10);

	SystemReset();
} 

/** ****************************************************************************
 * @name _UcbReadApp
 * @brief Handles UCB read application command
 * @param [in] port - number request came in on, the reply will go out this port
 * @param [out] packetPtr - data part of packet
 * @retval N/A
 ******************************************************************************/
static void _UcbReadApp(ExternPortTypeEnum port, UcbPacketStruct  *ptrUcbPacket)
{
    uint32_t startAddress;
    uint8_t  wordsToRead;
    uint16_t bytesToRead;

    startAddress = (uint32_t)((ptrUcbPacket->payload[0] << 24) |
                               ptrUcbPacket->payload[1] << 16 |
                               ptrUcbPacket->payload[2] << 8 |
                               ptrUcbPacket->payload[3]);
    wordsToRead = ptrUcbPacket->payload[4];
    bytesToRead = (uint16_t)wordsToRead;

    SetMaxDelay_Watchdog();

    /// verify that the packet length matches packet specification
    if (bytesToRead <= UCB_APP_MAX_LENGTH && (startAddress + bytesToRead) <= 0x80000) {
        readFlash(startAddress, &ptrUcbPacket->payload[5], bytesToRead);
        ptrUcbPacket->payloadLength = bytesToRead + 5;
    } else {
        _SetNak(port, ptrUcbPacket);
    }
	HandleUcbTx(port, ptrUcbPacket);
    RestoreDelay_Watchdog();	
}



/** ****************************************************************************
 * @name HandleUcbPacket - API
 * @brief general handler
 * Trace: [SDD_HANDLE_PKT <-- SRC_HANDLE_PACKET]
 * @param [in] port - port type UCB or CRM
 * @param [out] packetPtr - filled in packet from the mapped physical port
 * @retval N/A
 ******************************************************************************/
void HandleUcbPacket ( UcbPacketStruct *ptrUcbPacket)
{
    int result;

    ExternPortTypeEnum port = userSerialChan;
    if(port >= 0 && port < NUM_UART_PORTS && ptrUcbPacket){

		switch (ptrUcbPacket->packetType) {
            case UCB_PING:
                _UcbPing(port, ptrUcbPacket); break;
            case UCB_ECHO:
                _UcbEcho(port, ptrUcbPacket); break;
            case UCB_GET_PACKET:
                _UcbGetPacket(port, ptrUcbPacket); break;
            case UCB_SET_FIELDS:
                _UcbSetFields(port, ptrUcbPacket); break;
            case UCB_GET_FIELDS:
                _UcbGetFields(port, ptrUcbPacket); break;
            case UCB_READ_FIELDS:
                _UcbReadFields(port, ptrUcbPacket); break;
            case UCB_WRITE_FIELDS:
                _UcbWriteFields(port, ptrUcbPacket); break;
            case UCB_UNLOCK_EEPROM:
                _UcbUnlockEeprom(port, ptrUcbPacket); break;
            case UCB_LOCK_EEPROM:
                _UcbLockEeprom(port, ptrUcbPacket); break;
            case UCB_READ_EEPROM:
                _UcbReadEeprom(port, ptrUcbPacket); break;
            case UCB_READ_APP:
                _UcbReadApp(port, ptrUcbPacket); break;
            case UCB_WRITE_EEPROM:
                _UcbWriteEeprom(port, ptrUcbPacket); break;
            case UCB_PROGRAM_RESET:
                _UcbProgramReset(port, ptrUcbPacket); break;
            case UCB_SOFTWARE_RESET:
                _UcbSoftwareReset(port, ptrUcbPacket); break;
            case UCB_WRITE_CAL:
                _UcbWriteCal(port, ptrUcbPacket); break;
			case UCB_JUMP2_IAP:
				_UcbJump2IAP(port, ptrUcbPacket);break;
            case UCB_ERROR_INVALID_TYPE:
                _UcbError(port, ptrUcbPacket); break;
            case UCB_ERROR_TIMEOUT:
                _UcbError(port, ptrUcbPacket); break;
            case UCB_ERROR_CRC_FAIL:
                _UcbError(port, ptrUcbPacket); break;
#ifndef CAN_BUS_COMM
            case UCB_USER_IN:
                result = HandleUserInputPacket(ptrUcbPacket);
                if(result != USER_PACKET_OK){
  		            _SetNak(port,ptrUcbPacket);
                }
                HandleUcbTx(port, ptrUcbPacket);
                if(fReset){
                    fReset = FALSE;
                    OS_Delay(10);
                    NVIC_SystemReset();
                }
                break;
#endif
            default:
                _UcbError(port, ptrUcbPacket); break;
                break;
		}
	}
}
/* end HandleUcbPacket() */

/** ****************************************************************************
 * @name SystemReset - API
 * @brief system reset
 * Trace: [SDD_HANDLE_PKT <-- SRC_HANDLE_PACKET]
 * @retval N/A
 ******************************************************************************/
void SystemReset(void)
{
	*((u32 *)0xE000ED0C) = 0x05fa0004; 
}

/** ****************************************************************************
 * @name _WriteMagAlignParamsToMemory
 * @brief writes the magnetic alignment parameters to the EEPROM
 * Trace: [SDD_HANDLE_PKT <-- SRC_HANDLE_PACKET]
 * @retval N/A
 ******************************************************************************/
void WriteMagAlignParamsToMemory( ExternPortTypeEnum port,
                                  UcbPacketStruct    *ptrUcbPacket )
{
    // Array sizes are based on maximum number of fields to change
    uint16_t fieldId   [UCB_MAX_PAYLOAD_LENGTH / 4];
    fieldId[0] = 0x0009;   // X-Axis Hard-Iron
    fieldId[1] = 0x000A;   // Y-Axis Hard-Iron
    fieldId[2] = 0x000B;   // Soft-Iron Scale-Ratio
    fieldId[3] = 0x000E;   // Soft-Iron Angle

    uint16_t fieldData [UCB_MAX_PAYLOAD_LENGTH / 4];
    fieldData[0] = gConfiguration.hardIronBias[0];      // [G]
    fieldData[1] = gConfiguration.hardIronBias[1];      // [G]
    fieldData[2] = gConfiguration.softIronScaleRatio;   // [N/A]
    fieldData[3] = gConfiguration.softIronAngle;        // [deg]

    uint8_t numFields = 0x4;
    ptrUcbPacket->payloadLength = 1 + 4*numFields;
    ptrUcbPacket->payload[0]    = numFields;

    // first two are field ID, second two are data
    for( uint8_t fieldNum = 0; fieldNum < numFields; fieldNum++ ) {
        ptrUcbPacket->payload[(fieldNum*numFields)+1] = fieldId[fieldNum] >> 8;
        ptrUcbPacket->payload[(fieldNum*numFields)+2] = fieldId[fieldNum] & 0x00FF;
        ptrUcbPacket->payload[(fieldNum*numFields)+3] = fieldData[fieldNum] >> 8;
        ptrUcbPacket->payload[(fieldNum*numFields)+4] = fieldData[fieldNum] & 0x00FF;
    }

    _UcbWriteFields( port, ptrUcbPacket );
}

/** ****************************************************************************
 * @name _ProcessUcbCommands
 *
 * @brief  This routine will test for the a port to be assigned to the UCB
 * function and test that port for a received packet.  If the packet is received
 * it will call a handler.
 *
 * Trace:
 * [SDD_PROCESS_USER_PORTS_01 <-- SRC_PROCESS_UCB_COMMANDS]
 * [SDD_PROCESS_USER_PORTS_02 <-- SRC_PROCESS_UCB_COMMANDS]
 * [SDD_PROCESS_USER_PORTS_03 <-- SRC_PROCESS_UCB_COMMANDS]
 * [SDD_PROCESS_COMMANDS_SEQ <-- SRC_PROCESS_UCB_COMMANDS]
 *
 * @param [In] N/A
 * @param [Out] N/A
 * @retval N/A
 ******************************************************************************/
void ProcessUserCommands (void)
{
    /// check received packets and handle appropriately
    HandleUcbRx (&primaryUcbPacket);

} /* end ProcessUcbCommands() */

