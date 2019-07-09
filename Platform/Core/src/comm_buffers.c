/** ***************************************************************************
 * @file comm_buffers.c
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 * @details
 * This is a set of routines to move data in and out of the communication
 * circular buffers. They will also report back the byte used and bytes available
 * in the buffer passed. These are common routines used on both bootloader and DUP
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

#include "comm_buffers.h"
#include "osapi.h"

#define GOOD TRUE
#define BAD  FALSE

/** ****************************************************************************
 * @name COM_buf_bytes_available
 * @brief returns the number of bytes IN the circular buffer the buf_struc
 *        points to.
 * Trace:
 * [SDD_COM_BUFFER_BYTES_02 <-- SRC_COM_BUFFER_BYTES]
 * [SDD_COM_BUFFER_BYTES_03 <-- SRC_COM_BUFFER_BYTES]
 * @param [in] *circBuf - pointer to the circular buffer structure
 * @retval number of bytes in the buffer
 ******************************************************************************/
unsigned int COM_buf_bytes_available (cir_buf_t *circBuf)
{
    return circBuf->bytes_in_buffer;
}   /* end of COM_buf_bytes_available */

/** ****************************************************************************
 * @name COM_buf_headroom
 * @brief returns the number of bytes AVAILABLE in the circular
 * buffer the buf_struc points
 * Trace:
 * [SDD_COM_BUF_SPACE_02 <-- SRC_COM_BUF_SPACE]
 * [SDD_COM_BUF_SPACE_03 <-- SRC_COM_BUF_SPACE]
 * @param [in] *circBuf - pointer to the circular buffer structure
 * @retval number of bytes available in the buffer.
 ******************************************************************************/
unsigned int COM_buf_headroom (cir_buf_t *circBuf)
{
	return (circBuf->buf_size - circBuf->bytes_in_buffer);
}   /* end of COM_buf_headroom */

/** ****************************************************************************
 * @name COM_buf_delete
 * @brief This routine moves the tail pointer by - adding popCnt to the output
 *        pointer of the circular buffer. If popCnt exceeds the number of bytes
 *        in the buffer nothing will be removed.
 * Trace:
 * [SDD_COM_BUF_POP_01 <-- SRC_COM_BUF_POP]
 * [SDD_COM_BUF_POP_02 <-- SRC_COM_BUF_POP]
 * [SDD_COM_BUF_POP_03 <-- SRC_COM_BUF_POP]
 * @param [in] *circBuf - pointer to the circular buffer structure
 * @param [in] delCnt - number of bytes to remove from buffer
 * @retval number of bytes popped.
@details Note: This does NOT decrement the bytes available
 ******************************************************************************/
unsigned int COM_buf_delete (cir_buf_t  *circBuf, unsigned int delCnt)
{

    OSDisableHook();
    
	if(circBuf->dma_bytes_to_tx != 0){
        // assuming that all bytes got transmitted
        delCnt = circBuf->dma_bytes_to_tx;
        circBuf->dma_bytes_to_tx = 0;
    }else if(delCnt > circBuf->bytes_in_buffer){
        delCnt = circBuf->bytes_in_buffer;
    }

	if (circBuf->bytes_in_buffer && delCnt)  {
		if (circBuf->bytes_in_buffer < delCnt)  {
			delCnt = circBuf->bytes_in_buffer;	// delete all
		}
		circBuf->bytes_in_buffer -= delCnt;
		circBuf->buf_outptr      += delCnt;
		circBuf->buf_outptr      &= circBuf->buf_size-1;	// assuming size if power of 2
    }else{
		delCnt = 0;
	} 
	OSEnableHook();
		return delCnt;
}  /* end of COM_buf_delete */

/** ****************************************************************************
 * @name COM_buf_delete_isr
 * @brief This routine moves the tail pointer by - adding popCnt to the output
 *        pointer of the circular buffer. If popCnt exceeds the number of bytes
 *        in the buffer nothing will be removed.
 * Trace:
 * [SDD_COM_BUF_POP_01 <-- SRC_COM_BUF_POP]
 * [SDD_COM_BUF_POP_02 <-- SRC_COM_BUF_POP]
 * [SDD_COM_BUF_POP_03 <-- SRC_COM_BUF_POP]
 * @param [in] *circBuf - pointer to the circular buffer structure
 * @param [in] delCnt - number of bytes to remove from buffer
 * @retval number of bytes popped.
@details Note: This does NOT decrement the bytes available
 ******************************************************************************/
unsigned int COM_buf_delete_isr (cir_buf_t  *circBuf, unsigned int delCnt)
{

	if(circBuf->dma_bytes_to_tx != 0){
        // assuming that all bytes got transmitted
        delCnt = circBuf->dma_bytes_to_tx;
        circBuf->dma_bytes_to_tx = 0;
    }else if(delCnt > circBuf->bytes_in_buffer){
        delCnt = circBuf->bytes_in_buffer;
    }

	if (circBuf->bytes_in_buffer && delCnt)  {
		if (circBuf->bytes_in_buffer < delCnt)  {
			delCnt = circBuf->bytes_in_buffer;	// delete all
		}
		circBuf->bytes_in_buffer -= delCnt;
		circBuf->buf_outptr      += delCnt;
		circBuf->buf_outptr      &= circBuf->buf_size-1;	// assuming size if power of 2
    }else{
		delCnt = 0;
	} 

	return delCnt;
}  /* end of COM_buf_delete */


/** ****************************************************************************
 * @name COM_buf_delete_byte
 * @brief This routine moves the tail pointer by - adding popCnt to the output
 *        pointer of the circular buffer. If popCnt exceeds the number of bytes
 *        in the buffer nothing will be removed.
 *
 * Trace:
 * [SDD_COM_BUF_POP_01 <-- SRC_COM_BUF_POP]
 * [SDD_COM_BUF_POP_02 <-- SRC_COM_BUF_POP]
 * [SDD_COM_BUF_POP_03 <-- SRC_COM_BUF_POP]
 *
 * @param [in] *circBuf - pointer to the circular buffer structure
 * @param [in] delCnt - number of bytes to remove from buffer
 * @retval number of bytes popped 1 or 0.
 ******************************************************************************/
unsigned int COM_buf_delete_byte (cir_buf_t *circBuf )
{
	unsigned int newOut;

	if ( circBuf->bytes_in_buffer >= 1 )  {
		circBuf->bytes_in_buffer -= 1;
		newOut      = circBuf->buf_outptr + 1;
		if(newOut >= circBuf->buf_size) {
			newOut -= circBuf->buf_size;
		}
		circBuf->buf_outptr = newOut;
		return 1;
    } // else
    return 0; // UNDERFLOW - do nothing
}  /* end of COM_buf_delete_byte */

/** ****************************************************************************
 * @name COM_buf_copy copy out the number of bytes requested without removing
 *       them from the buffer.
 * @brief This routine uses the outptr and the bufIndex to point to the start
 *        of a string of bytes. If asked for more bytes than are in the buffer
 *        an error will be generated. The bytes read use the bufOut pointer for
 *        the starting add.
 * Trace:
 * [SDD_COM_BUF_READ_01 <-- SRC_COM_BUF_READ]
 * [SDD_COM_BUF_READ_02 <-- SRC_COM_BUF_READ]
 * [SDD_COM_BUF_READ_03 <-- SRC_COM_BUF_READ]
 * [SDD_COM_BUF_READ_04 <-- SRC_COM_BUF_READ]
 * [SDD_COM_BUF_READ_05 <-- SRC_COM_BUF_READ]
 * @param [in] circBuf - pointer to the circular buffer structure
 * @param [in] bufIndex - buffer index
 * @param [in] cnt  - number of bytes output to bufOut pointer
 * @param [in] bufOut - points to the output string of chars
 * @retval status returns a BOOL either GOOD or BAD
 ******************************************************************************/
BOOL COM_buf_copy (cir_buf_t *circBuf,
                   unsigned int  bufIndex,
                   unsigned int  cnt,
                   unsigned char *bufOut)
{
	unsigned int strPtr;
	unsigned int i;

	strPtr = circBuf->buf_outptr + bufIndex;
    // if there is more bytes available than requested
	if(circBuf->bytes_in_buffer >= cnt + bufIndex) {
		for (i = 0; i < cnt; i++) {
			if (strPtr >= circBuf->buf_size) {
				strPtr -= circBuf->buf_size;
			}
			*bufOut = *(circBuf->buf_add + strPtr++);
		 	bufOut++;
	    }
		return GOOD;
	}
    return BAD; // UNDEREFLOW
}   /* end of COM_buf_copy */

/** ****************************************************************************
 * @name COM_buf_copy_byte copy out a byte without removing it from the buffer.
 * @brief This routine will use the outptr and the bufIndex to point to the start
 * of a string of charaters.  If asked for more characters than are in the
 * buffer an error will be generated. The bytes read use the bufOut pointer for
 * the starting add.
 * Trace:
 * [SDD_COM_BUF_READ_01 <-- SRC_COM_BUF_READ]
 * [SDD_COM_BUF_READ_02 <-- SRC_COM_BUF_READ]
 * [SDD_COM_BUF_READ_03 <-- SRC_COM_BUF_READ]
 * [SDD_COM_BUF_READ_04 <-- SRC_COM_BUF_READ]
 * [SDD_COM_BUF_READ_05 <-- SRC_COM_BUF_READ]
 * @param [in] circBuf - pointer to the circular buffer structure
 * @param [in] bufIndex - buffer index
 * @param [in] bufOut - points to the output string of chars
 * @retval status returns a BOOL either GOOD or BAD
 ******************************************************************************/
BOOL COM_buf_copy_byte (cir_buf_t     *circBuf,
						unsigned int  bufIndex,
						unsigned char *bufOut)
{
	unsigned int strPtr;

	strPtr = circBuf->buf_outptr + bufIndex;
    // if there are more bytes available than requested
	if(circBuf->bytes_in_buffer >= 1 + bufIndex) {
			if (strPtr >= circBuf->buf_size) {
				strPtr -= circBuf->buf_size;
			}
			*bufOut = *(circBuf->buf_add + strPtr++);
		 	bufOut++;
		return GOOD;
	}
    return BAD; // UNDEREFLOW
}   /* end of COM_buf_copy_byte */

/** ****************************************************************************
 * @name COM_buf_add - add the number of bytes requested to the circ buffer
 * @brief uses cnt to check the available space in the circular
 *        buffer. If there is enough room, write it into the buffer using the
 *        pointer passed to it. If there is not enough room in the buffer no
 *        data will be sent and an error will be returned
 * Trace:
 * [SDD_COM_BUF_IN_01 <-- SRC_COM_BUF_IN]
 * [SDD_COM_BUF_IN_02 <-- SRC_COM_BUF_IN]
 * [SDD_COM_BUF_IN_03 <-- SRC_COM_BUF_IN]
 * [SDD_COM_BUF_IN_04 <-- SRC_COM_BUF_IN]
 * @param [in] circBuf - pointer to the circular buffer structure
 * @param [in] *buf - pointer to input data
 * @param [in] cnt - number of bytes to put into the circular buffer
 * @param [out] circBuf.inptr in the buffer struct will be incremeted
 * @retval status returns a BOOL either GOOD or BAD
 ******************************************************************************/
int COM_buf_add(cir_buf_t  *circBuf, unsigned char *buf, unsigned int cnt)
{
	unsigned int i;
	int headroom   = COM_buf_headroom(circBuf); 

	if(headroom < cnt){
		cnt = headroom;
	}

	for (i = 0; i < cnt; i++)
	{
			*(circBuf->buf_add + (circBuf->buf_inptr)) = *buf;
			circBuf->buf_inptr++;
            buf++;
		// carefull here - buffer considered to be power of 2 length
		circBuf->buf_inptr &= (circBuf->buf_size - 1);
		}
		circBuf->bytes_in_buffer += cnt;
	return cnt;
}   /* end of COM_buf_add */



/** ****************************************************************************
 * @name COM_buf_get copy and remove "Pop" the number of bytes requested
 * @brief Compares the number of bytes in cnt to the number of
 *        bytes in the buffer.  If the cnt is greater than the number of bytes
 *        in the buffer then no data will be moved and an error returned. If
 *        there is enough data in the circular buffer then cnt number of bytes
 *        will be moved to the location pointed to by the buf pointer.
 * Trace:
 * [SDD_COM_BUF_OUT_01 <-- SRC_COM_BUF_OUT]
 * [SDD_COM_BUF_OUT_02 <-- SRC_COM_BUF_OUT]
 * [SDD_COM_BUF_OUT_03 <-- SRC_COM_BUF_OUT]
 * [SDD_COM_BUF_OUT_04 <-- SRC_COM_BUF_OUT]
 * @param [in] *circBuf - pointer to the circular buffer structure
 * @param [in] *buf - pointer to the output location
 * @param [in] cnt - number of byte to pull from cir buffer.
 * @param [out] circBuf.outptr in the buffer is incremented
 * @retval status returns either GOOD [1] or BAD [0] set if not enough bytes in
 *         circular buffer.
 ******************************************************************************/

int COM_buf_get(cir_buf_t   *circBuf, unsigned char *buf, unsigned int  cnt)
{
	unsigned int i, num;
	num = circBuf->bytes_in_buffer;

    if(cnt > num){
	    cnt = num;
    }

    for (i = 0; i < cnt; i++) {
        *buf = *(circBuf->buf_add + (circBuf->buf_outptr));
        buf++;
        circBuf->buf_outptr++;
        circBuf->buf_outptr &= circBuf->buf_size-1;	// size is power of 2
    }
	OSDisableHookIfNotInIsr();
	circBuf->bytes_in_buffer -= cnt;
	OSEnableHookIfNotInIsr();
	return cnt;
}       /*end of COM_buf_get*/

/** ****************************************************************************
 * @name COM_buf_prepare_dma_transaction
 * @brief returns the number of bytes to be transmitted by dma engine and 
 *        pointer to start byte to transmit.
 * Trace:
 * [SDD_COM_BUFFER_BYTES_02 <-- SRC_COM_BUFFER_BYTES]
 * [SDD_COM_BUFFER_BYTES_03 <-- SRC_COM_BUFFER_BYTES]
 * @param [in] *circBuf - pointer to the circular buffer structure
 * @retval number of bytes in the buffer
 ******************************************************************************/
unsigned int COM_buf_prepare_dma_tx_transaction (cir_buf_t *circBuf, uint8_t **dataBufPtr)
{
    circBuf->dma_bytes_to_tx = circBuf->buf_size - circBuf->buf_outptr;
    if(circBuf->dma_bytes_to_tx > circBuf->bytes_in_buffer){
        circBuf->dma_bytes_to_tx = circBuf->bytes_in_buffer;
    }
    *dataBufPtr = circBuf->buf_add + circBuf->buf_outptr; 
    return circBuf->dma_bytes_to_tx;
}   /* end of COM_buf_bytes_available */
