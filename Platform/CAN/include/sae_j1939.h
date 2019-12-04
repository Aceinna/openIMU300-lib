/** ***************************************************************************
 * @file sae_j1939.h definitions of SAE J1939 standard
 * @Author Feng
 * @date   Feb. 2017
 * @brief  Copyright (c) 2017 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *****************************************************************************/
#ifndef SAE_J1939_H
#define SAE_J1939_H
#include "stm32f4xx_can.h"
#include "can.h"


typedef enum {
  GLOBAL,
  ON_HIGHWAY_EQUIPMENT,
  AGRICULTURE_FORESTRY_EQUIPMENT,
  CONSTRUCTION_EQUIPMENT,
  MARINE,
  INDUSTRY_PROCESS,
  RESERVED
} SAE_J1939_INDUSTRY_GROUP;  

typedef enum {
  ARBITRARY_ADDRESS_DISBAlE,
  ARBITRARY_ADDRESS_ENABLE
} ARBITRARY_ADDRESS_CAPABLE;  


// J1939 Name definition
#define ACEINNA_SAE_J1939_VEHICLE_SYSTEM               0
#define ACEINNA_SAE_J1939_VEHICLE_SYSTEM_INSTANCE      0
#define ACEINNA_SAE_J1939_FUNCTION                     131
#define ACEINNA_SAE_J1939_FUNCTION_INSTANCE            0
#define ACEINNA_SAE_J1939_ECU                          0
#define ACEINNA_SAE_J1939_MANUFACTURER_CODE            823


// J1939 64-bit name structure
typedef union {
  struct {
    uint64_t arbitrary_address       : 1;     // arbit bits
    uint64_t industry_group          : 3;     // group bits
    uint64_t vehicle_system_instance : 4;     // system instance bits
    uint64_t vehicle_system          : 7;     // system bits
    uint64_t reserved                : 1;
    uint64_t function                : 8;     // function bits
    uint64_t function_instance       : 5;     // function instance bits
    uint64_t ecu                     : 3;     // ecu bits
    uint64_t manufacture_code        : 11;    // manufacture code from SAE, 823 belongs to Aceinna
    uint64_t identity_number         : 21;    // id bits
  } bits;
  
  uint64_t words;
} SAE_J1939_NAME_FIELD;

// J1939 29-bit identifier
typedef struct {
    uint8_t source;               // source address
    uint8_t pdu_specific;         // ps value
    uint8_t pdu_format;           // pf value
    union {
        struct {
        uint8_t data_page  : 1;   // data page bit
        uint8_t ext_page   : 1;   // extended tata page
        uint8_t priority   : 3;   // priority bits
        uint8_t reserved   : 3;
    } control_bits;
    uint8_t r;
  };
} SAE_J1939_IDENTIFIER_FIELD;


// J1939 PF definition, see protocol spec
// Broadcast PFs - 240 to 255
#define SAE_J1939_PDU_FORMAT_DATA                    240
#define SAE_J1939_PDU_FORMAT_241                     241
#define SAE_J1939_PDU_FORMAT_248                     248
#define SAE_J1939_PDU_FORMAT_ECU                     253
#define SAE_J1939_PDU_FORMAT_254                     254	
#define SAE_J1939_PDU_FORMAT_251                     251	
#define SAE_J1939_PDU_FORMAT_GLOBAL                  255

#define SAE_J1939_PDU_FORMAT_ACK                     232
#define SAE_J1939_PDU_FORMAT_REQUEST                 234
#define SAE_J1939_PDU_FORMAT_ADDRESS_CLAIM           238



// J1939 PS definition, see protocol spec
#define SAE_J1939_PDU_SPECIFIC_25                    25
#define SAE_J1939_PDU_SPECIFIC_243                   243	// 65267 Vehicle Position 1
#define SAE_J1939_PDU_SPECIFIC_191                   191	// 65215 Wheel Speed Information
#define SAE_J1939_PDU_SPECIFIC_232                   232	// 65256 Vehicle Direction/Speed
#define SAE_J1939_PDU_SPECIFIC_1                     1
#define SAE_J1939_PDU_SPECIFIC_2                     2
#define SAE_J1939_PDU_SPECIFIC_246                   246	// 654502 Vehicle GNSS DOP
#define SAE_J1939_PDU_SPECIFIC_110                   110	// 65390  CUSTOM_GNSS_TIME
#define SAE_J1939_PDU_SPECIFIC_111                   111	// 65391  CUSTOM_GNSS_FIX
#define SAE_J1939_PDU_SPECIFIC_112                   112	// 65392  CUSTOM_GNSS_DOP


// J1939 PS definition
#define SAE_J1939_GROUP_EXTENSION_ECU                197
#define SAE_J1939_GROUP_EXTENSION_SOFTWARE_VERSION   218
#define SAE_J1939_GROUP_EXTENSION_ALGORITHM_RESET    80
#define SAE_J1939_GROUP_EXTENSION_SAVE_CONFIGURATION 81
#define SAE_J1939_GROUP_EXTENSION_TEST_HARDWARE      82
#define SAE_J1939_GROUP_EXTENSION_TEST_SOFTWARE      83
#define SAE_J1939_GROUP_EXTENSION_TEST_STATUS        84
#define SAE_J1939_GROUP_EXTENSION_PACKET_RATE        85
#define SAE_J1939_GROUP_EXTENSION_PACKET_TYPE        86
#define SAE_J1939_GROUP_EXTENSION_DIGITAL_FILTER     87
#define SAE_J1939_GROUP_EXTENSION_ORIENTATION        88
#define SAE_J1939_GROUP_EXTENSION_USER_BEHAVIOR      89
#define SAE_J1939_GROUP_EXTENSION_BUILT_IN_TEST      90
#define SAE_J1939_GROUP_EXTENSION_ANGLE_ALARM        91
#define SAE_J1939_GROUP_EXTENSION_CONE_ALARM         92
#define SAE_J1939_GROUP_EXTENSION_ACCELERATION_PARAM 93
#define SAE_J1939_GROUP_EXTENSION_MAG_ALIGN_CMD      94
#define SAE_J1939_GROUP_EXTENSION_BANK0              240
#define SAE_J1939_GROUP_EXTENSION_BANK1              241
#define SAE_J1939_GROUP_EXTENSION_ADDR               254
#define SAE_J1939_GROUP_EXTENSION_ACK                255
#define SAE_J1939_GROUP_EXTENSION_VP                 243


#define SAE_J1939_GROUP_EXTENSION_SLOPE_SENSOR       41      
#define SAE_J1939_GROUP_EXTENSION_ANGULAR_RATE       42
#define SAE_J1939_GROUP_EXTENSION_ACCELERATION       45
#define SAE_J1939_GROUP_EXTENSION_MAG                106

// J1939 message priority
#define SAE_J1939_CONTROL_PRIORITY                   6
#define SAE_J1939_REQUEST_PRIORITY                   6
#define SAE_J1939_ACCELERATION_PRIORITY              2
#define SAE_J1939_SLOPE_PRIORITY                     3
#define SAE_J1939_POSITION_PRIORITY                  6
#define SAE_J1939_ATTITUDE_PRIORITY                  6
#define SAE_J1939_RUPDATE_PRIORITY                   6

// J1939 message page number, currently uses 0
#define SAE_J1939_DATA_PAGE                          0


// MTLT message type upon J1939, from protocol spec
typedef enum {
    ACEINNA_SAE_J1939_VERSION                    = 1,      
    ACEINNA_SAE_J1939_ECU_ID                     = 2,
    ACEINNA_SAE_J1939_ALGORITHM_RESET            = 3,
    ACEINNA_SAE_J1939_CONFIG_SAVE                = 4,
    ACEINNA_SAE_J1939_ACK                        = 5,
    ACEINNA_SAE_J1939_HARDWARE_BITS              = 6,
    ACEINNA_SAE_J1939_SOFTWARE_BITS              = 7,
    ACEINNA_SAE_J1939_STATUS                     = 8,
    ACEINNA_SAE_J1939_RATE_DIVIDER               = 9,
    ACEINNA_SAE_J1939_PACKET_TYPE                = 10,
    ACEINNA_SAE_J1939_DIGITAL_FILTER             = 11,
    ACEINNA_SAE_J1939_ORIENTATION                = 12,
    ACEINNA_SAE_J1939_USER_BEHAVIOR              = 13,
    ACEINNA_SAE_J1939_BUILT_IN_TEST              = 14,
    ACEINNA_SAE_J1939_ANGLE_ALARM                = 15,
    ACEINNA_SAE_J1939_CONE_ALARM                 = 16,
    ACEINNA_SAE_J1939_ACCELERATION_PARAMETERS    = 17,
    ACEINNA_SAE_J1939_PS_BANK0                   = 18,
    ACEINNA_SAE_J1939_PS_BANK1                   = 19,
    ACEINNA_SAE_J1939_MAG_ALIGN_CMD              = 20,
} ACEINNA_SAE_J1939_CONTROL;

// MTLT message's length, see protocol spec
#define SAE_J1939_IDENTIFIER_LEN                       4
#define SAE_J1939_REQUEST_LEN                          3
#define SAE_J1939_PAYLOAD_MAX_LEN                      8
#define SAE_J1939_MAX_BUFFER_LEN                       20
#define SAE_J1939_REQUEST_LEN                          3
#define SAE_J1939_PAYLOAD_LEN_8_BYTES                  8
#define SAE_J1939_PAYLOAD_LEN_2_BYTES                  2

#define ACEINNA_SAE_J1939_VERSION_PACKET_LEN            6
#define ACEINNA_SAE_J1939_ECU_PACKET_LEN                8

#define ACEINNA_SAE_J1939_ALGO_RST_LEN                  3
#define ACEINNA_SAE_J1939_SAVE_CONFIG_LEN               3
#define ACEINNA_SAE_J1939_ACK_LEN                       8
#define ACEINNA_SAE_J1939_HARDWARE_BITS_LEN             2
#define ACEINNA_SAE_J1939_SOFTWARE_BITS_LEN             2
#define ACEINNA_SAE_J1939_STATUS_LEN                    2
#define ACEINNA_SAE_J1939_PACKET_RATE_LEN               2
#define ACEINNA_SAE_J1939_PACKET_TYPE_LEN               3
#define ACEINNA_SAE_J1939_DIGITAL_FILTER_LEN            3
#define ACEINNA_SAE_J1939_ORIENTATION_LEN               3
#define ACEINNA_SAE_J1939_USER_BEHAVIOR_LEN             3
#define ACEINNA_SAE_J1939_BUILT_IN_TEST                 8
#define ACEINNA_SAE_J1939_ANGLE_ALARM_LEN               7
#define ACEINNA_SAE_J1939_CONE_ALARM_LEN                7
#define ACEINNA_SAE_J1939_ACCELERATION_PARAM_LEN        7
#define ACEINNA_SAE_J1939_MAG_ALIGN_LEN                 8
#define ACEINNA_SAE_J1939_BANK0_LEN                     8
#define ACEINNA_SAE_J1939_BANK1_LEN                     8

#define ACEINNA_SAE_J1939_SLOPE_SENSOR2_LEN             8
#define ACEINNA_SAE_J1939_ANGULAR_RATE_LEN              8
#define ACEINNA_SAE_J1939_ACCELERATION_LEN              8
#define ACEINNA_SAE_J1939_MAG_LEN                       8
#define ACEINNA_SAE_J1939_POSITION_LEN                  8


// MTLT's operation 
#define ACEINNA_SAE_J1939_REQUEST                       0
#define ACEINNA_SAE_J1939_RESPONSE                      1

#define ACEINNA_SAE_J1939_SUCCESS                       0
#define ACEINNA_SAE_J1939_FAILURE                       1

// MTLT's address pool
#define ACEINNA_SAE_J1939_ADDRESS_MIN                 128
#define ACEINNA_SAE_J1939_ADDRESS_MAX                 247

// MTLT supports data types
#define ACEINNA_SAE_J1939_SLOPE_SENSOR_TYPE             1
#define ACEINNA_SAE_J1939_ANGULAR_RATE_TYPE             2
#define ACEINNA_SAE_J1939_ACCELERATOR_TYPE              4
#define ACEINNA_SAE_J1939_TYPE_MASK                     7

#define ACEINNA_SAE_J1939_SLOPE_OFFSET                  250.00
#define ACEINNA_SAE_J1939_XL_OFFSET                     320.00
#define ACEINNA_SAE_J1939_RATE_OFFSET                   250.00


// MTLT's ODR on CAN
enum {
  ACEINNA_SAE_J1939_PACKET_RATE_0           =           0,   //quiet
  ACEINNA_SAE_J1939_PACKET_RATE_2           =           2,   // 2Hz
  ACEINNA_SAE_J1939_PACKET_RATE_5           =           5,   // 5Hz
  ACEINNA_SAE_J1939_PACKET_RATE_10          =           10,  // 10Hz
  ACEINNA_SAE_J1939_PACKET_RATE_20          =           20,  // 20Hz
  ACEINNA_SAE_J1939_PACKET_RATE_25          =           25,  // 25Hz
  ACEINNA_SAE_J1939_PACKET_RATE_50          =           50,  // 50Hz
  ACEINNA_SAE_J1939_PACKET_RATE_100         =           100  // 100Hz
};


// MTLT's packet types
enum {
  ACEINNA_SAE_J1939_PACKET_SLOPE_SENSOR     =           0x01,   // slope sensor
  ACEINNA_SAE_J1939_PACKET_ANGULAR_RATE     =           0x02,   // angular rate
  ACEINNA_SAE_J1939_PACKET_ACCELERATION     =           0x04,   // acceleration
  ACEINNA_SAE_J1939_PACKET_MAG              =           0x08,   // mag
  ACEINNA_SAE_J1939_PACKET_LATLONG          =           0x10,   // Lattitude, longitude
  ACEINNA_SAE_J1939_PACKET_HEADING          =           0x20,   // Heading
  ACEINNA_SAE_J1939_PACKET_RAPID_COURSE_UPDATE      =           0x40,   // Rapid Course Update
  ACEINNA_SAE_J1939_PACKET_ATTITUDE         =           0x80,   // Attitude
  ACEINNA_SAE_J1939_PACKET_RAPID_POSITION_UPDATE    =           0x100   // Rapid position Update
};
 

// MTLT's connection status
typedef enum {
  _ECU_IDLE                  =   0,        // ready
  _ECU_LOST_CONNECTION       =   1,        // connection lost
  _ECU_NORMAL                =   2,        // normal state
  _ECU_EMPTY_ADDRESS         =   3,        // no address assigned
  _ECU_EXPIRED               =   4         // address expires
} _ECU_STATUS;

// MTLT's states
typedef enum {
  _ECU_INVALID_NAME          =   -1,       // invalid name
  _ECU_TX_OVERFLOW           =   -2,       // tx queue overflow
  _ECU_RX_OVERFLOW           =   -3,       // rx queue overflow
  _ECU_BAUDRATE_DETECT       =   1,        // baud rate detection
  _ECU_WAIT_ADDRESS          =   2,        // none of address
  _ECU_CHECK_ADDRESS         =   3,        // check address conflicted
  _ECU_ALGORITHM_RESET       =   4,        // algo reset
  _ECU_READY                 =   64,       // ready to be used
  _ECU_WAIT_ID               =   65,       // wait for ecu id packets
  _ECU_WAIT_SOFTWARE_VER     =   66,       // wait for software version packets
  _ECU_WAIT_ALG_RESET        =   68,       // wait for algo reset packet
  _ECU_WAIT_CONFIG_SAVE      =   72,       // wait config save packet
  _ECU_WAIT_BUILTIN_TEST     =   80        // wait for bit status packet
} _ECU_STATE;


// MTLT is used as slave or master device
typedef enum {
  _ECU_MASTER                =   0,        // host device
  _ECU_SLAVE                 =   1         // slave device
} _ECU_CATEGORY;


// MTLT's control packet types
typedef enum {
  SAE_J1939_REQUEST_PACKET            =    1,       // request packet
  SAE_J1939_ACK_PACKET                =    2,       // ack packet
  SAE_J1939_RESPONSE_PACKET           =    3,       // response packet
  SAE_J1939_SET_PACKET                =    4,       // set packet
  SAE_J1939_DATA_PACKET               =    5        // data packet
} SAE_J1939_PACKET_TYPE;


// MTLT's tranmitting desc
struct sae_j1939_tx_desc {
  SAE_J1939_PACKET_TYPE       tx_pkt_type;          // tx packet type
  uint8_t                     tx_payload_len;       // tx payload length
  DESC_STATE                  tx_pkt_ready;         // tx state
  SAE_J1939_IDENTIFIER_FIELD  tx_identifier;        // idnetifier of tx packet
  CanTxMsg                    tx_buffer;            // tx buffer

  struct sae_j1939_tx_desc *next;  
};


// MTLT's desc numbers
#define SAE_J1939_MAX_TX_DESC             32
#define SAE_J1939_MAX_RX_DESC             32


// MTLT's receive desc
struct sae_j1939_rx_desc {
  uint8_t                     rx_pkt_len;         // rx packet length
  DESC_STATE                  rx_pkt_ready;       // rx state
  SAE_J1939_IDENTIFIER_FIELD  rx_identifier;      // indentifier of rx packet
  CanRxMsg                    rx_buffer;          // rx buffer
  
  struct sae_j1939_rx_desc * next;
};

// software version packet type
typedef struct {
  SAE_J1939_IDENTIFIER_FIELD  ver_pgn;            // software version PGN
} VERSION_PACKET;


// ECU ID packet type
typedef struct {
  SAE_J1939_IDENTIFIER_FIELD ecu_id_pgn;          // ECU id PGN
  uint8_t *ecu_id;
} ECU_ID_PACKET;


// address claim type
typedef struct {
  SAE_J1939_IDENTIFIER_FIELD addr_claim_pg_pgn;    // address claim PGN
} ADDR_CLAIM_PG_PACKET;

// bit status definition, coming from MTLT's user's guide
#define ACEINNA_SAE_J1939_BUILTIN_HARDWARE               1
#define ACEINNA_SAE_J1939_BUILTIN_SOFTWARE               2
#define ACEINNA_SAE_J1939_BUILTIN_STATUS                 4



// Bank0 set payload format
typedef struct {
        uint8_t dest_address;              // destination address
        uint8_t alg_reset_ps;              // new ps value of alg reset
        uint8_t save_cfg_ps;               // new ps value of config save
        uint8_t status_ps;                 // new ps value of status message
        uint8_t mag_align_ps;              // new ps value of mag alignment cmd
        uint8_t rsvd1;                     // reserved
        uint8_t rsvd2;                     // reserved
        uint8_t rsvd3;                     // reserved
        SAE_J1939_IDENTIFIER_FIELD  bank0_pgn;
} BANK0_PS_PAYLOAD;


// Bank1 set payload format
typedef struct {
        uint8_t dest_address;              // destination address
        uint8_t packet_rate_ps;            // new ps value of packet rate
        uint8_t packet_type_ps;            // new ps value of packet type
        uint8_t digital_filter_ps;         // new ps value of lpf
        uint8_t orientation_ps;            // new ps value of orientation
        uint8_t user_behavior_ps;          // new ps value of user behavior
        uint8_t rsvd1;                     // reserved
        uint8_t rsvd2;                     // reserved
        SAE_J1939_IDENTIFIER_FIELD  bank1_pgn;
} BANK1_PS_PAYLOAD;


// slope sensor data payload format
typedef struct {
    uint64_t pitch                :       24;       // pitch 
    uint64_t roll                 :       24;       // roll
    uint64_t pitch_compensation   :       2;        // pitch compensation
    uint64_t pitch_merit          :       2;        // pitch merit
    uint64_t roll_compensation    :       2;        // roll compensation
    uint64_t roll_merit           :       2;        // roll merit
    uint64_t measure_latency      :       8;        // latency
} SLOPE_SENSOR_2;

// angular rate data payload format
typedef struct {
    uint16_t pitch_rate;                            // pitch rate
    uint16_t roll_rate;                             // roll  rate
    uint16_t yaw_rate;                              // yaw   rate
    uint8_t  pitch_merit          :       2;        // pitch rate merit
    uint8_t  roll_merit           :       2;        // roll  rate merit
    uint8_t  yaw_merit            :       2;        // yaw  rate merit
    uint8_t  rsvd                 :       2;        // rsvd
    uint8_t  measurement_latency;                   // latency
} AUGULAR_RATE;


// accleration data payload format
typedef struct {
    uint16_t   acceleration_x;                      // x-axis acceleration
    uint16_t   acceleration_y;                      // y-axis acceleration
    uint16_t   acceleration_z;                      // z-axis acceleration
    uint8_t    lateral_merit        :       2;      // laterar acc merit
    uint8_t    longitudinal_merit   :       2;      // longitudinal merit
    uint8_t    vertical_merit       :       2;      // vertical merit
    uint8_t    transmit_rate        :       2;      // repetition rate
    uint8_t    rsvd;
} ACCELERATION_SENSOR;

// accleration data payload format
typedef struct {
    uint16_t   mag_x;                        // x-axis mag data
    uint16_t   mag_y;                        // y-axis mag data
    uint16_t   mag_z;                        // z-axis mag data
    uint16_t   unuzed;                       
} MAGNETIC_SENSOR;

// gps position data payload format
// PGN 65267
// PGN 129025 
typedef struct {
    uint32_t    latitude;                            // gps latitude  0.0000001 deg/bit
    uint32_t    longitude;                           // gps longitude 0.0000001 deg/bit
} GPS_DATA;

// Vehicle Direction/Speed
// PGN 65256
typedef struct {
    int16_t    compassBearing;    // 0.0078125  deg/bit
    int16_t    navSpeed;          // 0.00390625 kph/bit
    int16_t    pitch;             // 0.0078125  deg/bit
    int16_t    altitude;          // 0.125      m/bit
}DIR_SPEED_DATA;

// Vehicle GNSS dop
// PGN 64502
typedef struct {
    int8_t     numSats;           // Number Of Sattelites
    int8_t     HDOP;              // Horizontal Dilution of Precision - 1 count/bit
    int8_t     VDOP;              // Vertical Dilution of Precision   - 0.1 / bit
    int8_t     PDOP;              // Position Dilution of Precision   - 0.1 / bit
    int8_t     TDOP;              // Time Dilution of Precision       - 0.1 / bit
    int8_t     rsvd1;             // 
    int8_t     rsvd2;             // 
    int8_t     rsvd3;             // 
}GNSS_DOP_DATA;


// Wheel speed data
// PGN 65215
#pragma pack (1)
typedef struct {
    int16_t    frontAxleSpeed;                     // 0.00390625 kph/bit
    int8_t     relSpeedFrontAxleLeftWheel;         // 0.0625 km/h per bit
    int8_t     relSpeedFrontAxleRightWheel;        // 0.0625 km/h per bit 
    int8_t     relSpeedRearAxle1LeftWheel;         // 0.0625 km/h per bit
    int8_t     relSpeedRearAxle1RigthWheel;        // 0.0625 km/h per bit
    int8_t     relSpeedRearAxle2LeftWheel;         // 0.0625 km/h per bit
    int8_t     relSpeedRearAxle2RigthWheel;        // 0.0625 km/h per bit 
}WHEEL_SPEED_DATA;

  
// COG & SOG, Rapid Update
// PGN  129026
typedef struct{
	uint8_t 	SID;
	uint8_t 	COG_Reference : 2;		// 0 - TRUE, 1 - Magnetic
	uint8_t 	Reserved      : 6;		// 0x3f
	int16_t 	COG;                    // course over ground  0.0001 rad/bit  
	int16_t 	SOG;                    // speed  over ground  0.01   m/s/ bit
	int16_t 	Rsvd;   				// 0xFFFF             
}COURSE_RAPID_UPDATE_DATA;

// Attitude
// PGN 127257
typedef struct{
	uint8_t SID;
	int16_t Yaw;						// 0.0001 rad/bit
	int16_t Pitch;						// 0.0001 rad/bit
	int16_t Roll;						// 0.0001 rad/bit
	uint8_t Rsvd;
}ATTITUDE_DATA;

// GNSS Custom message 1
// PGN 65392
typedef struct{
    uint16_t hAcc;                      // /1000 m, if > 65535 -> 65535 , horizontal accuracy estimate
    uint16_t vAcc;                      // /1000 m, if > 65535 -> 65535 , vertical accuracy estimate
    uint16_t pDOP;                      // ++ scaling is 0.01, position DOP
    int16_t  headMot;                   // *182/100000 deg, 
}CUSTOM_GNSS_DOP_DATA;

// GNSS Custom message 2
// PGN 65391
typedef struct{
    uint8_t numSV;                      //  number of satellites in Nav solution;
    uint8_t flags;                      //  
    uint8_t valid;                      // Validity flags
    uint8_t fixType;                    // GNSS fix type
    int32_t height;                     // mm, height above ellipsoid;
}CUSTOM_GNSS_FIX_DATA;

// GNSS Custom message 2
// PGN 65391
typedef struct{
    uint32_t iTOW;                      // ms
    int32_t gSpeed;                     // mm/s, groud speed (2-D)
}CUSTOM_GNSS_TIME_DATA;


#pragma pack ()

#define SAE_J1939_MAX_TABLE_ENTRY         128

// address table
typedef struct {
  	SAE_J1939_NAME_FIELD ecu_name;             	  // ecu's name
  	uint8_t  address;                             // ecu's address
  	_ECU_STATUS status;                        	  // ecu's status
  	_ECU_CATEGORY  category;                   	  // ecu's category
  	uint32_t last_scan_time; // ms                // time of last message sent
  	uint32_t idle_time;    // ms                  // idle time
  	uint32_t alive_time;   // second              // alive time
} ECU_ADDRESS_ENTRY;


// angular rate data payload format
typedef struct {
    uint64_t    request        : 8;
    uint64_t    status         : 8;
    uint64_t    hardIronX      : 12;    // 8/2048   = 0.004   Gauss/bit
    uint64_t    hardIronY      : 12;    // 8/2048   = 0.004   Gauss/bit
    uint64_t    softIronSF     : 10;    // 1/1024   = 0.001   / bit
    uint64_t    softIronAngle  : 14;    // 180/2048 ~ 0.01    deg/bit
} MAG_ALIGN_RESPONSE;


// CAN's configuration parameters
typedef struct {
  SAE_J1939_NAME_FIELD ecu_name;            // ecu's name
  uint8_t  address;                         // ecu's address
  uint8_t  baud_rate_detect_enable;         // auto detection enable/disable
  uint16_t baudRate;                        // baud rate
  uint8_t  version[5];                      // software version
  uint16_t packet_rate;                     // odr
  uint16_t packet_type;                     // packet type
  uint8_t  accel_cut_off;                   // Xl's lpf
  uint8_t  rate_cut_off;                    // rate's lpf
  uint16_t  orien_bits;                     // orientation
  uint8_t  restart_on_overrange;            // restart on over range
  uint8_t  dynamic_motion;                  // dynamic motion
  uint8_t roll_upper;                       // upper limit of roll
  uint8_t roll_lower;                       // lower limit of roll
  uint8_t pitch_upper;                      // upper limit of pitch
  uint8_t pitch_lower;                      // lower limit of pitch
  uint8_t roll_hysteresis;                  // hysteresis of roll
  uint8_t pitch_hyseresis;                  // hysteresis of pitch
  uint16_t alarm_selector;                  // alarm selector
  uint16_t angle_limit;                     // angular limit
  uint16_t angle_hysteresis;                // angular hysteresis
  uint16_t acceleration_x;                  // x-axis acceleration
  uint16_t acceleration_y;                  // y-axis acceleration
  uint16_t acceleration_z;                  // z-axis acceleration
  uint8_t  config_changed;                  // confiugration changed
  uint8_t alg_reset_ps;                     // new ps value of alg reset
  uint8_t save_cfg_ps;                      // new ps value of config save
  uint8_t status_ps;                        // new ps value of status message
  uint8_t packet_rate_ps;                   // new ps value of packet rate
  uint8_t packet_type_ps;                   // new ps value of packet type
  uint8_t digital_filter_ps;                // new ps value of lpf
  uint8_t orientation_ps;                   // new ps value of orientation
  uint8_t user_behavior_ps;                 // new ps value of user behavior
  uint8_t mag_align_ps;                     // new ps value of mag align command
} EcuConfigurationStruct;


// ECU structure
typedef struct {
  SAE_J1939_NAME_FIELD        *name;          // ecu's name
  uint8_t                     *addr;          // ecu's address
  _ECU_CATEGORY               category;       // ecu's category
  _ECU_STATE                  state;          // ecu's state
  
  ECU_ADDRESS_ENTRY           * addrTbl;      // address table
  
  struct sae_j1939_tx_desc    * curr_tx_desc;               // current tx desc
  struct sae_j1939_rx_desc    * curr_process_desc;          // current desc processed by hardware
  struct sae_j1939_rx_desc    * curr_rx_desc;               // current rx desc
  
  void                        (* init_table)(void);         // initilize address table
  void                        (* update_table)(ECU_ADDRESS_ENTRY *entry);  // update adddress table
  uint8_t                     (* add_entry)(uint8_t, SAE_J1939_NAME_FIELD); // add new entry 
  uint8_t                     (* del_entry)(ECU_ADDRESS_ENTRY *);     // delete an entry
   
  uint8_t                     (* xmit)(struct sae_j1939_tx_desc *);   // transmitting function
 
} ECU_INSTANCE;

extern ECU_INSTANCE *gEcu;


// packet type mask
enum {
  _ECU_CONFIG_PACKET_RATE                   =      1,   // set packet rate
  _ECU_CONFIG_PACKET_TYPE                   =      2,   // set packet type
  _ECU_CONFIG_DIGITAL_FILTER                =      4,   // set lpf
  _ECU_CONFIG_ORIENTATION                   =      8,   // set orientation
  _ECU_CONFIG_USER_BEHAVIOR                 =     16,   // set user behavior
  _ECU_CONFIG_ANGLE_ALARM                   =     32,   // set angular alarm
  _ECU_CONFIG_CONE_ALARM                    =     64,   // set cone alarm
  _ECU_CONFIG_ACCELERATION_PARAM            =    128,   // set acceleration parameters
  _ECU_CONFIG_GROUP_EXTENSION_BANK          =    256,   // set new ps
  _ECU_CONFIG_MASK                          =    511    // configuration mask
};

// command types 

typedef enum {
    ACEINNA_J1939_INVALID_IDENTIFIER         =     -1,    // invalid indentifier
    ACEINNA_J1939_IGNORE                     =      0,    // ignore
    ACEINNA_J1939_SOFTWARE_VERSION           =      2,    // sofware version packet
    ACEINNA_J1939_ECU_ID                     =      3,    // ecu id packet
    ACEINNA_J1939_ALG_RST                    =      4,    // alg reset packet
    ACEINNA_J1939_CFG_SAVE                   =      5,    // config save packet
    ACEINNA_J1939_HARDWARE_TEST              =      6,    // hardware bit packet
    ACEINNA_J1939_SOFTWARE_TEST              =      7,    // software bit packet
    ACEINNA_J1939_STATUS_TEST                =      8,    // status packet
    ACEINNA_J1939_BUILTIN_TEST               =      9,    // built-in test packet
    ACEINNA_J1939_DATA                       =      10,   // data packet
    ACEINNA_J1939_ADDRESS_CLAIM              =      11,   // address claim packet
    ACEINNA_J1939_REQUEST_PACKET             =      12,   // request packet
    ACEINNA_J1939_CONFIG                     =      13    // config packet
} ACEINNA_J1939_PACKET_TYPE;

// ECU address status
enum {
  _ECU_ADDR_AVAILABLE                       =      0,    // available address value
  _ECU_ADDR_OCCUPIED                        =      1     // occupied address value
};

// ECU address structure
typedef struct {
  uint8_t      status;                 // status, available or occupied
  uint8_t      addr;                   // address
} ACEINNA_ECU_ADDR;

// set command type
typedef struct {
    uint8_t request;                              // request or response
    uint8_t dest_address;                         // target's address
    uint8_t success;                              // successful or failure
} COMMAND_SET_PAYLOAD;

typedef struct{
  	int 	pkt_type;
	int 	priority;
	uint8_t PF;
	uint8_t PS;
	uint8_t len;
	uint8_t source;
    uint8_t data_page;
    uint8_t ext_page;
}msg_params_t;



#define ACEINNA_ECU_ADDRESS_MAX              120

extern ECU_INSTANCE gEcuInst;
extern EcuConfigurationStruct gEcuConfig;
extern EcuConfigurationStruct *gEcuConfigPtr;

extern void sae_j1939_initialize(uint16_t baudRate, uint8_t address);

extern void initialize_mapping_table();
extern void update_mapping_table(ECU_ADDRESS_ENTRY *);
extern uint8_t del_ecu_mapping_table(ECU_ADDRESS_ENTRY *);

extern void sae_j1939_get_identifier(SAE_J1939_IDENTIFIER_FIELD *);
extern void sae_j1939_set_identifier(SAE_J1939_IDENTIFIER_FIELD *);
extern ACEINNA_J1939_PACKET_TYPE is_valid_j1939_master_recv(struct sae_j1939_rx_desc *);
extern uint8_t is_valid_sae_j1939_identifier(SAE_J1939_IDENTIFIER_FIELD *);

extern void ecu_process(void);
extern void ecu_transmit(void); 

extern uint8_t find_tx_desc(struct sae_j1939_tx_desc **);
extern ACEINNA_J1939_PACKET_TYPE is_valid_data_packet(SAE_J1939_IDENTIFIER_FIELD *);
extern ACEINNA_J1939_PACKET_TYPE is_valid_config_command(SAE_J1939_IDENTIFIER_FIELD *);
extern ACEINNA_J1939_PACKET_TYPE is_aceinna_data_packet(SAE_J1939_IDENTIFIER_FIELD *);\
extern ACEINNA_J1939_PACKET_TYPE is_algorithm_data_packet(SAE_J1939_IDENTIFIER_FIELD *);\
extern ACEINNA_J1939_PACKET_TYPE is_valid_address_claim(SAE_J1939_IDENTIFIER_FIELD *);

extern uint8_t send_j1939_packet(struct sae_j1939_tx_desc *);
extern void send_address_claim(ECU_INSTANCE *);
extern void build_set_pkt(struct sae_j1939_tx_desc *, ACEINNA_SAE_J1939_CONTROL, _ECU_CATEGORY);
extern void build_command_set(COMMAND_SET_PAYLOAD *, ECU_ADDRESS_ENTRY *, _ECU_CATEGORY);
extern void process_request_pg(struct sae_j1939_rx_desc *);
extern void build_request_pkt(struct sae_j1939_tx_desc *);
extern void process_address_claim(struct sae_j1939_rx_desc *desc);

extern void    aceinna_j1939_transmit_isr(void);
extern void    aceinna_j1939_receive_isr(void);
extern uint8_t aceinna_j1939_send_status_packet(uint8_t built_in_type, void * bit_fields);
extern uint8_t aceinna_j1939_send_software_version(void);
extern uint8_t aceinna_j1939_send_ecu_id(void);
extern uint8_t aceinna_j1939_send_slope_sensor(SLOPE_SENSOR_2 * data);
extern uint8_t aceinna_j1939_send_acceleration(ACCELERATION_SENSOR * data);
extern uint8_t aceinna_j1939_send_angular_rate(AUGULAR_RATE * data);
extern uint8_t aceinna_j1939_send_algrst_cfgsave(ECU_ADDRESS_ENTRY *target, uint8_t alg_rst, uint8_t success);
extern uint8_t aceinna_j1939_send_mag_align(uint8_t cmd, uint8_t state, real *params);
extern uint8_t aceinna_j1939_send_packet_rate(uint8_t odr);
extern uint8_t aceinna_j1939_send_packet_type(uint16_t type);
extern uint8_t aceinna_j1939_send_digital_filter(uint8_t accel_cutoff, uint8_t rate_cutoff);
extern uint8_t aceinna_j1939_send_orientation( uint8_t *orien);
extern uint8_t aceinna_j1939_send_mags(MAGNETIC_SENSOR * data);
extern uint8_t aceinna_j1939_send_GPS(GPS_DATA * data);
extern uint8_t aceinna_j1939_send_course_rapid_update(COURSE_RAPID_UPDATE_DATA * data);
extern uint8_t aceinna_j1939_send_attitude(ATTITUDE_DATA * data);
extern uint8_t aceinna_j1939_send_position_rapid_update(GPS_DATA * data);
extern uint8_t aceinna_j1939_build_msg(void *payload, msg_params_t *params);


extern void      SaveEcuAddress(uint16_t address);
extern BOOL      SaveEcuConfig(EcuConfigurationStruct  *gEcuConfigPtr);
extern uint8_t   GetEcuAddress();
extern int       GetEcuBaudRate();
extern void      SetEcuBaudRate(_ECU_BAUD_RATE rate );
extern void      SetEcuPacketType(uint16_t type);
extern void      SetEcuPacketRate(uint16_t rate);
extern void      SetEcuFilterFreq(EcuConfigurationStruct  *pEcuConfig);
extern void      SetEcuOrientation(uint16_t orien_bits);
extern BOOL      UpdateEcuConfig(EcuConfigurationStruct  *gEcuConfigPtr, BOOL fSave);
extern BOOL      CanTermResistorEnabled();
extern BOOL      CanBaudRateDetectionEnabled();
extern BOOL      UseAlgorithm();

// priority 6
// PF 254
// PS 0xA0 - 0xBF and 0xE0 - 0xFF
#define VEHICLE_DATA_ID_BASE          0x18FEA000
#define VEHICLE_DATA_FILTER_BASE_MASK 0x18FEA000

// priority 6
// PF 251
// PS 0xA0 - 0xBF and 0xE0 - 0xFF
#define VEHICLE_DATA1_ID_BASE          0x18FBF000
#define VEHICLE_DATA1_FILTER_BASE_MASK 0x18FBF000


// priority 6
// PF 255
// PS 240 and 241 
#define ACEINNA_BANK_ID_BASE          0x18FFF000
#define ACEINNA_BANK_FILTER_BASE_MASK 0x18FFF000


// priority 6
// PF 238
// PS 255 
#define SAE_J1939_ADDRESS_CLAIM_ID_BASE          0x18EEFF00
#define SAE_J1939_ADDRESS_CLAIM_FILTER_BASE_MASK 0x18FFFF00

// priority 6
// PF 234
// PS 255  - global requests
#define SAE_J1939_REQUEST_ID_BASE          0x18EAFF00
#define SAE_J1939_REQUEST_FILTER_BASE_MASK 0x18EAFF00

// priority 6
// PF 255
// PS 60 - 6F
//#define SAE_J1939_CONTROL2_ID_BASE          0x18FFFF00
//#define SAE_J1939_CONTROL2_FILTER_BASE_MASK 0x18FF6000

// priority 6
// PF 255
// PS 50 - 5F
#define SAE_J1939_CONTROL1_ID_BASE          0x18FFFF00
#define SAE_J1939_CONTROL1_FILTER_BASE_MASK 0x18FF5000

// priority 6
// PF 253
// PS 197
#define SAE_J1939_ECU_ID_BASE               0x18FDC500
#define SAE_J1939_ECU_FILTER_BASE_MASK      0x18FDC500


#endif // SAE_J1939_H