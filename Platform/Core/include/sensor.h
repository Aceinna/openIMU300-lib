/** ***************************************************************************
 * @file   sensors.h
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

#ifndef SENSOR_H
#define SENSOR_H

#include "GlobalConstants.h"

#define ACCEL_FILTER_TAPS 9
#define GYRO_FILTER_TAPS 9

#define MAX_COVAR 6


#define RMAG_SERIAL_NUMBER_SIZE 4
#define RMAG_VERSION_STRING_SIZE 60
#define RMAG_MODEL_STRING_SIZE 20

// for sensor state variable 
#define M_ACCEL 				0x004
#define M_GYRO 					0x008
#define M_MAG 				  0x80000
#define M_AIR 				 0x100000
#define M_REMOTEMAG 		 0x200000

// for sensor algoState variable 
#define M_INIT              	0x1
#define M_TURN              	0x2
#define M_FORCED_VG   		    0x4

// for HeadingReset
#define TRUE_AND_USE_REMOTE_MAG      0x1
#define TRUE_AND_USE_EXT_HEADING     0x2

extern volatile unsigned char ioupDataStart;
extern unsigned char gIOUPSync;

#define NOSYNC	0
#define SYNCED	1
#define ZEROMSEC_CNT 0
#define ONEMSEC_CNT	 1
#define NINEMSEC_CNT 9
#define TENMSEC_CNT	10
#define TWENTYONEMSEC_CNT  21
#define SHIFT2BYTES	16
#define MAX_DELTA_PKT_CNT (100)
#define INITIOUPCOMM_TIMEOUT 1000

// IOUP version string xmitted from IOUP
// 15 char + null max: "xxxx-xxxx-xx_xx"   
#define IOUP_VERSION_SIZE 15                // # chars in version string 
extern char gIoupVersion[IOUP_VERSION_SIZE+1];
//for ADAHRS, software version is added   
#define IOUP_PN_VERSION_SIZE 20                // # chars in PN +version string 
extern char gIoupPNversion[IOUP_PN_VERSION_SIZE];


#define AIRDATADROPOUTMAX 500		// 5 seconds 
#define REMOTEMAGDROPOUTMAX 200		// 2 seconds 
#define VER_HDR 0xAA
#define SENSOR_PACKET_SIZE 16
#define	PKTCHKSUM 15             	// location of packet checksum 
#define IOUP_TIMEOUT_RESET  10      // timer loops to wait for IOUP to reset 
#define IOUP_MAX_COMM_ERR   500

#define SIGNED_TO_UNSIGNED(x) (((x) + 0x8000) & 0xFFFF)

// IOUP flips ADC counts before sending them out, need to flip them back 
#define REVERSE_ADC_COUNTS(counts)	(65535 - counts)

extern unsigned int packet[SENSOR_PACKET_SIZE];
extern unsigned int gIOUPpktcnt;

#define SENSOR_SCALING_FOR_RAW_ADC  7       // set the same as that for VG440 
#define TEMP_SCALING_FOR_RAW_ADC    14      // set the same as that for VG440 

extern int16_t LimitInt16Value( int16_t value, int16_t limit );


#endif // SENSOR_H 

