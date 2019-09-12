/** ***************************************************************************
 * @file can.h control area network functions
 * @Author Feng
 * @date   May 2017
 * @brief  Copyright (c) 2017 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *****************************************************************************/
#ifndef __CAN_H
#define __CAN_H
#include <stdint.h>
#include "GlobalConstants.h"
//#include "stm32f4xx.h"
//#include "stm32f4xx_can.h"

// supported baud rate
typedef enum {
  _ECU_500K      =    0,                    // 500kbps
  _ECU_250K      =    1,                    // 250kbps
  _ECU_125K      =    2,                    // 125kbps
  _ECU_1000K     =    3                     // 1000kbps
} _ECU_BAUD_RATE;

// MTLT's state machine
typedef enum {
  DESC_IDLE                  =   0,        // ready for being used
  DESC_OCCUPIED              =   1,        // unavailable
  DESC_PENDING               =   2         // in queue
} DESC_STATE;

// MTLT's ODR on CAN
enum {
  CAN_PACKET_RATE_0           =           0,   //quiet
  CAN_PACKET_RATE_2           =           2,   // 2Hz
  CAN_PACKET_RATE_5           =           5,   // 5Hz
  CAN_PACKET_RATE_10          =           10,  // 10Hz
  CAN_PACKET_RATE_20          =           20,  // 20Hz
  CAN_PACKET_RATE_25          =           25,  // 25Hz
  CAN_PACKET_RATE_50          =           50,  // 50Hz
  CAN_PACKET_RATE_100         =           100, // 100Hz
  CAN_PACKET_RATE_200         =           200  // 200Hz
};


#define CAN_ERROR                          -1
#define CAN_NO_ERROR                        1

#define USER_CAN_IDE                 1
#define USER_CAN_RTR                 0

#define CAN_BAUD_RATE_RETRY          4          // retry times for baud rate auto detection
#define CAN_DETECT_TIME              3000       // ms, listening period at each of channels

extern uint32_t canRxIntCounter, canStartDetectRxIntCounter; 

extern void _CAN_Configure(void (*callback1)(void), void(*callback2)(void));
extern BOOL  CAN_Detect_Baudrate(_ECU_BAUD_RATE *rate);

#endif
