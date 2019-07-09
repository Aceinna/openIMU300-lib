/** ***************************************************************************
 * @file   uart.c
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *  UART driver for the DMU380's two user serial ports
 *	transmitting and receive of serial data and then number of bytes remaining in
 *	the circular buffer. There is no FIFO on the uart as there is in the 525
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

#ifndef __UART_H
#define __UART_H
#include "GlobalConstants.h"
//#include "boardDefinition.h"


#ifdef __cplusplus
extern "C" {
#endif    

extern int          uart_init(int uartChannel, int baudrate);
extern int          uart_read(int channel, uint8_t *data, int length);
extern int          uart_write(int channel, uint8_t *data, int length);
extern int          uart_rxBytesAvailable(int channel);
extern void         uart_flushRecBuffer(int channel);
extern int          uart_txBytesRemains(int channel);
extern int          uart_removeRxBytes(int gUartChannel, int numToPop);
extern int          uart_copyBytes(int channel, int index, int number, uint8_t *output);
extern void         uart_registerRxSemaphore(int portType, void *id);
extern void         uart_BIT(int uartType);
extern void         uart_Pause();
extern int          uart_bufferTx(int channel, uint8_t *data, int len);
extern void         uart_flashTxBuffer(int channel);

#ifdef __cplusplus
}
#endif    

#endif
