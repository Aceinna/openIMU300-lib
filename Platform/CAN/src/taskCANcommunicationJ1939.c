/** ***************************************************************************
 * @file   taskCANCommunication.c
 * @Author
 * @date   Aug, 2017
 * @brief  Copyright (c) 2017 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * End user communication with the DMU380:
 * - Nav-View (UART) maximum rate of 100 Hz (due to wait in the function below)
 *    handled by the Memsic ucb (Unified Code Base) - handle_packet, send_packet,
 *    extern_port.c, ucb_packet.c comm_buffers.c
 * - SPI communication is an external interrupt driven (asynchronous) bus
 ******************************************************************************/
#ifdef SAE_J1939

#include "debug.h"
#include "platformAPI.h"
#include "osresources.h"
#include "indices.h"
#include "UserConfiguration.h"
#include "UserMessagingCAN.h"
#include "sensors_data.h"


   
// for can interface
#include "canAPI.h"
#include "sae_j1939.h"

   
#define ADDRESS_CLAIM_RETRY                 5

extern uint32_t tim5_heart_beat;

extern uint32_t userCommunicationType;
extern void setUserCommunicationType(uint32_t);
uint32_t PacketRateCounter = 0;
uint32_t CanLoopCounter    = 0;
BOOL canStarted            = FALSE;

_ECU_BAUD_RATE baudRate; 
#define SAE_J1939_HEART_BEAT 2  // packets at 100 Hz

/** ***************************************************************************
 * @name TaskCANCommunication() CAN communication task
 * @brief  perform a thread of CAN transmission and receiving
 *
 * @param [in] 
 * @retval
 ******************************************************************************/
void TaskCANCommunicationJ1939(void const *argument)
{
    static int sendPeriodicPackets = 0;
    int res;
    _ECU_BAUD_RATE baudRate = (_ECU_BAUD_RATE)gEcuConfig.baudRate;
    int              address    = GetEcuAddress();
    BOOL  finished;
    
    InitCANBoardConfiguration_GPIO();
    InitCommunication_UserCAN(baudRate);

    // initialize J1939 protocol stack   
    if (!(gEcuConfigPtr->ecu_name.words)) {
        gEcuConfigPtr->ecu_name.bits.function         = ACEINNA_SAE_J1939_FUNCTION;
        gEcuConfigPtr->ecu_name.bits.manufacture_code = ACEINNA_SAE_J1939_MANUFACTURER_CODE;
        gEcuConfigPtr->ecu_name.bits.identity_number = unitSerialNumber() & 0x1fffff;
        gEcuConfigPtr->ecu_name.bits.arbitrary_address = 1; // support
    }
    
    sae_j1939_initialize(baudRate, address);
    
    // deactivate semaphore if active 
    res = osSemaphoreWait(canDataSem, 0);
    // Main loop for the task
    while( 1 )
    {  
        // wait for events from DACQ task
        // Semaphore given at 200Hz
        res = osSemaphoreWait(canDataSem, 1000);
        if(res != osOK){
            continue;
        }
        
        sendPeriodicPackets = 0;
        CanLoopCounter++;

        // Auto-detection 
        if (gEcuInst.state == _ECU_BAUDRATE_DETECT) {
            canStartDetectRxIntCounter =  canRxIntCounter;
            finished = CAN_Detect_Baudrate(&baudRate);
            if(!finished){
                continue;
            }
            gEcuInst.state = _ECU_CHECK_ADDRESS; 
        }

        canStarted = TRUE;        

        // address claiming state
        if ((gEcuInst.state == _ECU_CHECK_ADDRESS) || (gEcuInst.state == _ECU_WAIT_ADDRESS)|| !(CanLoopCounter % 200)) {
            if (CanLoopCounter > ADDRESS_CLAIM_RETRY){
                gEcuInst.state = _ECU_READY;
            }
            send_address_claim(&gEcuInst);
        }
        // process incoming messages
        ecu_process();

        uint64_t  dts = platformGetDacqTimeStamp();
        uint64_t  cts = platformGetCurrTimeStamp();
          // prepare outgoing data packets
        int latency   = (cts - dts)/100;    // in 0.1 ms  

        PacketRateCounter++;
        
        if(PacketRateCounter >= gEcuConfig.packet_rate*2){
            PacketRateCounter   = 0;
            sendPeriodicPackets = 1;
            // prepare and send outgoing periodic data packets
        }

        EnqeuePeriodicDataPackets(latency,sendPeriodicPackets);
        
        
        ecu_transmit(); 
        
    }
}
#endif // CAN_J1939
