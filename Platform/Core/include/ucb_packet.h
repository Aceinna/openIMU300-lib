/** ***************************************************************************
  * @file ucb_packet.h utility functions for interfacing with Memsic proprietary
  *       UCB (unified code base) packets.  UCB packet structure
  *
  * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY O ANY
  * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
  * PARTICULAR PURPOSE.
  *
  * @brief these are in ucb_packet_types.def on the 440 these were in
  *        xbowProtocol.h
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

#ifndef UCB_PACKET_H
#define UCB_PACKET_H

//#include "crc.h"
#include "GlobalConstants.h"
#include "ucb_packet_struct.h"

#define MATCHES 0

//  Xbow Packet Code     
typedef enum {
    UCB_PING,               //  0         
    UCB_ECHO,               //  1         
    UCB_GET_PACKET,         //  2         
    UCB_SET_FIELDS,         //  3         
    UCB_GET_FIELDS,         //  4         
    UCB_READ_FIELDS,        //  5         
    UCB_WRITE_FIELDS,       //  6         
    UCB_UNLOCK_EEPROM,      //  7         
    UCB_READ_EEPROM,        //  8         
    UCB_WRITE_EEPROM,       //  9         
    UCB_PROGRAM_RESET,      // 10         
    UCB_SOFTWARE_RESET,     // 11         
    UCB_WRITE_CAL,          // 12         
    UCB_JUMP2_IAP,          // 13         
    UCB_LOCK_EEPROM,        // 14         
    UCB_READ_APP,           // 15         
    UCB_INPUT_PACKET_MAX,   // 16
//**************************************************
    UCB_IDENTIFICATION,     // 16         
    UCB_VERSION_DATA,       // 17         
    UCB_VERSION_ALL_DATA,   // 18         
    UCB_SCALED_0,           // 19   0x16      
    UCB_SCALED_1,           // 20         
    UCB_TEST_0,             // 21         
    UCB_TEST_1,             // 22         
    UCB_FACTORY_1,          // 23   0x1a       
    UCB_FACTORY_2,          // 24         
    UCB_MAG_CAL_1_COMPLETE, // 25         
    UCB_MAG_CAL_3_COMPLETE, // 26      
    UCB_MAG_CAL_COMPLETE,    
    UCB_ANGLE_2,
    UCB_PKT_NONE,           // 27   marker after last valid packet 
    UCB_NAK,                // 28
    UCB_ERROR_TIMEOUT,      // 29         
    UCB_ERROR_CRC_FAIL,     // 30 
//NUM_UCB_PACKET_TYPE_ENUM, // 43 
} UcbPacketType;

extern UcbPacketStruct gSpiUcbPacket; 

//
#define UCB_IDENTIFICATION_LENGTH		69
#define UCB_VERSION_DATA_LENGTH			 5
#define UCB_VERSION_ALL_DATA_LENGTH		15
#define UCB_ANGLE_1_LENGTH				32
#define UCB_ANGLE_2_LENGTH				30
#define UCB_ANGLE_3_LENGTH				52
#define UCB_ANGLE_4_LENGTH			    42 // std A4 = 38 this is custom
#define UCB_ANGLE_5_LENGTH			    62
//#define UCB_ANGLE_U_LENGTH			    42
#define UCB_SCALED_0_LENGTH			    30
#define UCB_SCALED_1_LENGTH			    24
#define UCB_SCALED_3_LENGTH			    54
#define UCB_TEST_0_LENGTH				28
#define UCB_TEST_1_LENGTH				32
#define UCB_FACTORY_1_LENGTH			54
#define UCB_FACTORY_2_LENGTH			66
#define UCB_FACTORY_3_LENGTH            38
#define UCB_FACTORY_4_LENGTH			54
#define UCB_FACTORY_5_LENGTH			70
#define UCB_FACTORY_6_LENGTH			66
#define UCB_FACTORY_7_LENGTH           134
#define UCB_MAG_CAL_1_COMPLETE_LENGTH	 4
#define UCB_MAG_CAL_3_COMPLETE_LENGTH	10
#define UCB_MAG_CAL_COMPLETE_LENGTH     10
#define UCB_NAV_0_LENGTH			    32
#define UCB_NAV_1_LENGTH			    42
#define UCB_NAV_2_LENGTH			    46 // with ITOW
#define UCB_APP_MAX_LENGTH              240



/// UCB packet-specific utility functions ucb_packet.c
extern UcbPacketType     UcbPacketBytesToPacketType    (const uint8_t bytes []);
extern void              UcbPacketPacketTypeToBytes    (UcbPacketType type, uint8_t bytes []);
extern uint8_t           UcbPacketBytesToPayloadLength (const uint8_t bytes []);
extern void              UcbPacketPayloadLengthToBytes (uint8_t type, uint8_t bytes []);
extern UcbPacketCrcType  UcbPacketBytesToCrc           (const uint8_t bytes []);
extern void              UcbPacketCrcToBytes           (const UcbPacketCrcType crc, uint8_t bytes []);
extern UcbPacketCrcType  UcbPacketCalculateCrc         (const uint8_t data [], uint16_t length, const UcbPacketCrcType seed);
extern BOOL              UcbPacketIsAnInputPacket      (UcbPacketType type);
extern BOOL              UcbPacketIsAnOutputPacket     (UcbPacketType type);
extern void              UcbPacketPacketTypeToCode     (UcbPacketType type, uint16_t *code);

// send_packet.c
extern void SendUcbPacket   (UcbPacketStruct *ptrUcbPacket);
// handle packet.c
extern void HandleUcbPacket (UcbPacketStruct *ptrUcbPacket);
extern int  HandleUserInputPacket (UcbPacketStruct *ptrUcbPacket);
extern BOOL HandleUserOutputPacket (uint8_t *payload, uint8_t *payloadLen);
extern void SystemReset(void);

// Function used to write the Mag-Align parameters to the EEPROM by field
extern void WriteMagAlignParamsToMemory( uint16_t        port,
                                         UcbPacketStruct *ptrUcbPacket );
#endif
