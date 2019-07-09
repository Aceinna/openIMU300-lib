/** ***************************************************************************
 * @file port_def.h communications port data structures used by Uart and
 *                  GPS parsing routines
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

#ifndef PORT_DEF_H
#define PORT_DEF_H

#include "comm_buffers.h"

typedef struct{
    unsigned int  baud;  	  ///< com port baud rate index
    unsigned char tx_int_lvl; ///< com port tx FIFO interrupt level
    unsigned char rx_int_lvl; ///< com port rx FIFO interrupt level
//    unsigned char tx_int_flg; ///< 0 = transmit int off,1 = transmit int enabled
} uart_hw;

typedef struct{
    unsigned char datatype;
    unsigned int  rec_timeout;
} chan;

typedef struct{
    uart_hw	hw;        	///< UART hardware dependent variables
    chan 	cdef;       ///< COM channel dependent variables
    cir_buf_t	rec_buf;	///< Receive buffer array of pointers
    cir_buf_t	xmit_buf;   ///< Transmit buffer array of pointers
    int         txBusy;
    int         rxBusy;
} port_struct;

#endif

