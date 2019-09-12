/** ***************************************************************************
 * @file extern_port.c functions for general external port interface
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * @brief UCB (Unified Code Base) external serial interface
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
#include "GlobalConstants.h"
#include "serial_port.h"
#include "uart.h"
#include "crc16.h"
#include "ucb_packet.h"
#include "platformAPI.h"

int userSerialChan = UART_CHANNEL_0;


/// packet-type receive working buffer sizes
#define UCB_RX_WORKING_BUFFER_SIZE	128
#define CRM_RX_WORKING_BUFFER_SIZE	128

/// main loop period
extern const uint16_t MAIN_LOOP_PERIOD;		///< 10 milliseconds (approximately)


/// port behavior parameters
#define RX_POLL_PERIOD	MAIN_LOOP_PERIOD	///< milliseconds (approximately)
#define UCB_RX_TIMEOUT 	1000				///< milliseconds (approximately)
#define CRM_RX_TIMEOUT	1000				///< milliseconds (approximately)

typedef struct{
    int      type;
    uint16_t code;
}ucbInputSyncTableEntry_t;

/// List of allowed input packet codes 
ucbInputSyncTableEntry_t ucbInputSyncTable[] = {		//       
    {UCB_PING,               0x504B},   //  "PK" 
    {UCB_ECHO,               0x4348},   //  "CH" 
    {UCB_GET_PACKET,         0x4750},   //  "GP" 
    {UCB_SET_FIELDS,         0x5346},   //  "SF" 
    {UCB_GET_FIELDS,         0x4746},   //  "GF" 
    {UCB_READ_FIELDS,        0x5246},   //  "RF" 
    {UCB_WRITE_FIELDS,       0x5746},   //  "WF" 
    {UCB_UNLOCK_EEPROM,      0x5545},   //  "UE" 
    {UCB_LOCK_EEPROM,        0x4C45},   //  "LE" 
    {UCB_READ_EEPROM,        0x5245},   //  "RE" 
    {UCB_WRITE_EEPROM,       0x5745},   //  "WE" 
    {UCB_SOFTWARE_RESET,     0x5352},   //  "SR" 
    {UCB_WRITE_CAL,          0x5743},   //  "WC" 
    {UCB_JUMP2_IAP,          0x4A49},   //  "JI" 
    {UCB_READ_APP,           0x5241},   //  "RA" 
    {UCB_INPUT_PACKET_MAX,   0x00000000},    //  "  "
};

uint8_t dataBuffer[100];

/** ****************************************************************************
 * @name HandleUcbRx
 * @brief handles received ucb packets
 * Trace:
 *	[SDD_UCB_TIMEOUT_01 <-- SRC_HANDLE_UCB_RX]
 *	[SDD_UCB_PACKET_CRC <-- SRC_HANDLE_UCB_RX]
 *	[SDD_UCB_CONVERT_DATA <-- SRC_HANDLE_UCB_RX]
 *	[SDD_UCB_STORE_DATA <-- SRC_HANDLE_UCB_RX]
 *	[SDD_UCB_UNKNOWN_01 <-- SRC_HANDLE_UCB_RX]
 *	[SDD_UCB_CRC_FAIL_01 <-- SRC_HANDLE_UCB_RX]
 *	[SDD_UCB_VALID_PACKET <-- SRC_HANDLE_UCB_RX]
 *
 * @param [in] port - logical port type
 * @param [out] packetPtr - UCB packet to read the packet into
 * @retval TRUE if a full packet has been seen (can fail CRC)
 *         FALSE if needing more to fill in a packet
 ******************************************************************************/
BOOL HandleUcbRx (UcbPacketStruct  *ucbPacket)

{
    static int bytesInBuffer = 0, state = 0, crcError = 0, len = 0;
    static uint8_t *ptr;
    static uint16_t crcMsg = 0, code;
	static uint32_t sync = 0;
    unsigned char tmp;
	unsigned int  pos = 0, synced = 0, type;
	uint16_t crcCalc;
    ucbInputSyncTableEntry_t *syncTable;
	
    
	while(1){
        if(!bytesInBuffer){
            bytesInBuffer = uart_read(userSerialChan, dataBuffer, sizeof (dataBuffer));
            if(!bytesInBuffer){
                return 0; // nothing to do
            }
            pos = 0; 
        }
        tmp = dataBuffer[pos++];
        bytesInBuffer--;
        sync   = (sync << 8) | tmp;
        synced = 0;
        if((sync & 0xFFFF0000) == 0x55550000){
            code = sync & 0xffff;
            syncTable = ucbInputSyncTable;
            while (syncTable->type != UCB_INPUT_PACKET_MAX)
            {
                if (syncTable->code == code){
                    synced = 1;
                    type   = syncTable->type;
                    break;
                }
                syncTable++;
            }
#ifndef USER_PACKETS_NOT_SUPPORTED
            if(!synced){
                type = checkUserPacketType(code);
                if(type != UCB_ERROR_INVALID_TYPE){
                    synced = 1;
                }
            }
#endif
        }
        if(synced){
            ucbPacket->packetType    = type;
            ucbPacket->payloadLength = 0;
            ucbPacket->code_MSB      = (sync >> 8) & 0xff;
            ucbPacket->code_LSB      = sync & 0xff;
        	state  = 1;
		    len    = 0;
            synced = 0;
            continue;
        }
        switch(state){
        case 0:
            break;
        case 1:
            ucbPacket->payloadLength = tmp;
            if(tmp == 0){
                state = 3;  // crc next
            }else{
                state = 2;  // data next
                len   = 0;
            }
            ptr   = ucbPacket->payload;
            break;
        case 2:
            if(len++ > UCB_MAX_PAYLOAD_LENGTH){
                state = 0;
                break;
            }
            *ptr++ = tmp;
            if(len == ucbPacket->payloadLength){
                //crc next
                state  = 3;
                crcMsg = 0; 
            }
            break;
        case 3:
            crcMsg = tmp;
            *ptr++ = tmp;   
            state = 4;
            break;
        case 4:
            state   = 0;
            crcMsg  = crcMsg | ((uint16_t)tmp << 8);
            *ptr++  = tmp;   
            crcCalc = CalculateCRC((uint8_t*)&ucbPacket->code_MSB, len + 3);
            if(crcMsg != crcCalc){
                crcError++;
            }else {
                // process message here
               HandleUcbPacket (ucbPacket);
               platformUpdateDebugPortAssignment();
               return 0;   // will come back later
            }
            break;
        default:
            while(1){}; // should not be here
        }
    }

}
/* end HandleUcbRx */

/** ****************************************************************************
 * @name HandleUcbTx
 * @brief builds a UCB packet and then triggers transmission of it. Packet:
 *  Preamble = 0x5555
 *  Packet Type 0x####
 *  Length 0x##
 *  payload (uint8_t)data[Length]
 *  CRC 0x####
 * Trace: [SDD_UCB_PROCESS_OUT <-- SRC_UCB_OUT_PKT]
 * @param [in] port - port type UCB or CRM
 * @param [in] packetPtr -- buffer structure with payload, type and size
 * @retval valid packet in packetPtr TRUE
 ******************************************************************************/
void HandleUcbTx (int port, UcbPacketStruct *ptrUcbPacket)

{

	UcbPacketCrcType crc;
	uint8_t          data[2];

	/// get byte representation of packet type, index adjust required since sync
    /// isn't placed in data array
	UcbPacketPacketTypeToBytes(ptrUcbPacket->packetType, data);

	ptrUcbPacket->sync_MSB = 0x55;
	ptrUcbPacket->sync_LSB = 0x55;
	ptrUcbPacket->code_MSB = data[0];
	ptrUcbPacket->code_LSB = data[1];

    crc = CalculateCRC((uint8_t *)&ptrUcbPacket->code_MSB, ptrUcbPacket->payloadLength + 3);
    ptrUcbPacket->payload[ptrUcbPacket->payloadLength+1]   = (crc >> 8) & 0xff;
    ptrUcbPacket->payload[ptrUcbPacket->payloadLength]     =  crc  & 0xff;

    uart_write(userSerialChan, (uint8_t*)&ptrUcbPacket->sync_MSB, ptrUcbPacket->payloadLength + 7);

}
/* end HandleUcbTx */
