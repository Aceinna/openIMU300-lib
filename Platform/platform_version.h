/*****************************************************************************
 * @file platform_version.h
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * @brief Version definition based on UCB serial protocol.
 ******************************************************************************/
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
#ifndef _PLATFORM_300_VERSION_H
#define _PLATFORM_300_VERSION_H



// DO NOT CHANGE THESE NUMBERS FROM ZERO!  CAUSES A CONFLICT WITH
//   IMUTest RESULTING IN ACCELEROMETER VALUES THAT ARE FLIPPED (WHAT
//   SHOULD BE POSITIVE BECOMES NEGATIVE).
#define VERSION_MAJOR 0
#define VERSION_MINOR 0
#define VERSION_PATCH 0
#define VERSION_STAGE 0
#define VERSION_BUILD 0


//
//                                    1         2
//                           12345678901234567890
#ifndef  CAN_BUS_COMM
    #define VERSION_MAJOR_NUM 04
    #define VERSION_MINOR_NUM 1
    #define VERSION_PATCH_NUM 0
    #define VERSION_STAGE_NUM 0
    #define VERSION_BUILD_NUM 2
    #define SOFTWARE_PART      "5020-3886-02 04.01.02"
#else
    #define VERSION_MAJOR_NUM 03
    #define VERSION_MINOR_NUM 1
    #define VERSION_PATCH_NUM 0
    #define VERSION_STAGE_NUM 0
    #define VERSION_BUILD_NUM 0
    #define  SOFTWARE_PART      "5020-3309-01 03.01.09"
#endif

#define  SOFTWARE_PART_LEN  30
#define  VERSION_STR        SOFTWARE_PART  
#define  N_VERSION_STR      128
#define  SIZEOF_WORD        2 // [bytes]
#define  SPI_SW_VERSION     41

/**
 * 'VR' and 'VA' only use UINT8 field bootloader reads unspecified UINT (but
 *  assumed 2 cells wide) and sensor_init() assumes 16 bits
 */
typedef struct {
    unsigned int major;
    unsigned int minor;
    unsigned int patch;
    unsigned int stage;
    unsigned int build;
} softwareVersionStruct;



extern softwareVersionStruct dupFMversion;  /// 525 digital processor DUP code base
extern softwareVersionStruct ioupFMversion; /// 525 input output processor IOUP code base
extern softwareVersionStruct bootFMversion; /// bootloader code base
extern softwareVersionStruct calVersion;    /// date and time of calibration



#endif
