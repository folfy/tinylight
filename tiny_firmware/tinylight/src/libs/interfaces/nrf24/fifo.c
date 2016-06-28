/*####COPYRIGHTBEGIN####
 -------------------------------------------
 Copyright (C) 2004 by Steve Karg

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 as published by the Free Software Foundation; either version 2
 of the License, or (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program; if not, write to:
 The Free Software Foundation, Inc.
 59 Temple Place - Suite 330
 Boston, MA  02111-1307
 USA.

 As a special exception, if other files instantiate templates or
 use macros or inline functions from this file, or you compile
 this file and link it with other works to produce a work based
 on this file, this file does not by itself cause the resulting
 work to be covered by the GNU General Public License. However
 the source code for this file must still be made available in
 accordance with section (3) of the GNU General Public License.

 This exception does not invalidate any other reasons why a work
 based on this file might be covered by the GNU General Public
 License.
 -------------------------------------------
####COPYRIGHTEND####*/

/* Functional Description: Generic FIFO library for deeply
   embedded system. See the unit tests for usage examples. */

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include "fifo.h"

/****************************************************************************
* DESCRIPTION: Returns the number of elements in the ring buffer
* RETURN:      Number of elements in the ring buffer
* ALGORITHM:   none
* NOTES:       none
*****************************************************************************/
uint8_t FIFO_Count (
    FIFO_BUFFER const *b)
{
    return (b ? (b->head - b->tail) : 0);
}

/****************************************************************************
* DESCRIPTION: Returns the number of free slots in the ring buffer
* RETURN:      Number of free slots in the ring buffer
* ALGORITHM:   none
* NOTES:       none
*****************************************************************************/
uint8_t FIFO_Free (
	FIFO_BUFFER const *b)
{
	return (b ? (b->buffer_len - FIFO_Count(b)) : 0);
}

/****************************************************************************
* DESCRIPTION: Returns the empty/full status of the ring buffer
* RETURN:      true if the ring buffer is full, false if it is not.
* ALGORITHM:   none
* NOTES:       none
*****************************************************************************/
static bool FIFO_Full (
    FIFO_BUFFER const *b)
{
    return (b ? (FIFO_Count(b) == b->buffer_len) : true);
}

/****************************************************************************
* DESCRIPTION: Returns the empty/full status of the ring buffer
* RETURN:      true if the ring buffer is empty, false if it is not.
* ALGORITHM:   none
* NOTES:       none
*****************************************************************************/
bool FIFO_Empty(
        FIFO_BUFFER const *b)
{
    return (b ? (FIFO_Count(b) == 0) : true);
}

/****************************************************************************
* DESCRIPTION: Looks at the data from the head of the list without removing it
* RETURN:      byte of data, or zero if nothing in the list
* ALGORITHM:   none
* NOTES:       Use Empty function to see if there is data to retrieve
*****************************************************************************/
uint8_t FIFO_Peek(
        FIFO_BUFFER const *b)
{
    if (b) {
        return (b->buffer[b->tail % b->buffer_len]);
    }

    return 0;
}

/****************************************************************************
* DESCRIPTION: Gets the data from the front of the list, and removes it
* RETURN:      the data, or zero if nothing in the list
* ALGORITHM:   none
* NOTES:       Use Empty function to see if there is data to retrieve
*****************************************************************************/
uint8_t FIFO_Get(
        FIFO_BUFFER * b)
{
    uint8_t data_byte = 0;

    if (!FIFO_Empty(b)) {
        data_byte = b->buffer[b->tail % b->buffer_len];
        b->tail++;
    }
    return data_byte;
}

/****************************************************************************
* DESCRIPTION: Get multiple element of data from the FIFO
* RETURN:      true on success, false if not
* ALGORITHM:   none
* NOTES:       none
*****************************************************************************/
bool FIFO_Getn(
	FIFO_BUFFER * b,
	uint8_t data_byte[],
	uint8_t count)
{
	uint8_t n;
	if(FIFO_Count(b)<count) {
		return false;
	}
	
	for(n=0; n<count; n++) {
		data_byte[n]=FIFO_Get(b);
	}
	
	return true;
}

/****************************************************************************
* DESCRIPTION: Adds an element of data to the FIFO
* RETURN:      true on successful add, false if not added
* ALGORITHM:   none
* NOTES:       none
*****************************************************************************/
bool FIFO_Put(
        FIFO_BUFFER * b,
        uint8_t data_byte)
{
    bool status = false;        /* return value */

    if (b) {
        /* limit the ring to prevent overwriting */
        if (!FIFO_Full(b)) {
            b->buffer[b->head % b->buffer_len] = data_byte;
            b->head++;
            status = true;
        }
    }

    return status;
}

/****************************************************************************
* DESCRIPTION: Adds multiple elements of data to the FIFO
* RETURN:      true on successful add, false if not added
* ALGORITHM:   none
* NOTES:       none
*****************************************************************************/
bool FIFO_Putn(
	FIFO_BUFFER * b,
	uint8_t data_byte[],
	uint8_t count)
{
	uint8_t n;
	if (FIFO_Free(b)<count) {
		return false;
	}

	for(n=0; n<count; n++) {
		FIFO_Put(b,data_byte[n]);
	}

	return true;
}

/****************************************************************************
* DESCRIPTION: Configures the ring buffer
* RETURN:      none
* ALGORITHM:   none
* NOTES:        buffer_len must be a power of two
*****************************************************************************/
void FIFO_Init(
    FIFO_BUFFER * b,
    uint8_t *buffer,
    uint8_t buffer_len)
{
    if (b) {
        b->head = 0;
        b->tail = 0;
        b->buffer = buffer;
        b->buffer_len = buffer_len;
    }

    return;
}
