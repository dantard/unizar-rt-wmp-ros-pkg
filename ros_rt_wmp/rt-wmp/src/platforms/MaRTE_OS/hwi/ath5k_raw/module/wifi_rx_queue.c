/*----------------------------------------------------------------------------
 *-------------------------      M a R T E   O S      ------------------------
 *----------------------------------------------------------------------------
 *   Copyright (C) 2003-2005   Universidad de Cantabria, SPAIN
 *
 *   MaRTE OS web page: http://marte.unican.es
 *
 *  MaRTE OS  is free software; you can  redistribute it and/or  modify it
 *  under the terms of the GNU General Public License  as published by the
 *  Free Software Foundation;  either  version 2, or (at  your option) any
 *  later version.
 *
 *  MaRTE OS  is distributed  in the  hope  that  it will be   useful, but
 *  WITHOUT  ANY  WARRANTY;     without  even the   implied   warranty  of
 *  MERCHANTABILITY  or  FITNESS FOR A  PARTICULAR PURPOSE.    See the GNU
 *  General Public License for more details.
 *
 *  You should have received  a  copy of  the  GNU General Public  License
 *  distributed with MaRTE  OS;  see file COPYING.   If not,  write to the
 *  Free Software  Foundation,  59 Temple Place  -  Suite 330,  Boston, MA
 *  02111-1307, USA.
 *
 *  As a  special exception, if you  link this  unit  with other  files to
 *  produce an   executable,   this unit  does  not  by  itself cause  the
 *  resulting executable to be covered by the  GNU General Public License.
 *  This exception does  not however invalidate  any other reasons why the
 *  executable file might be covered by the GNU Public License.
 *
 *---------------------------------------------------------------------------
 *
 *                            'r x _ q u e u e . c'
 *
 *                                     C
 *
 *
 * File 'rx_queue.c'                                                  By Chema.
 *                                                          Jose Maria Martinez
 *                                                            <chema@gmx.net>
 * Body fuctions of the ring queue (see description).
 *---------------------------------------------------------------------------*/

/* This module implements a ring buffer. Concurrent access is avoid if     */
/* exactly one producer and exactly one consumer are accesing. The reading */
/* process is waiting to consume data that is produced at interrupt time.  */
/* An important consideration of the ring buffer is that we consider the   */
/* buffer empty when the head (current_pos) is == tail (free_pos), and full*/
/* when the (tail + 1) % RING_BUFFER_LEN == head (we waste one element)    */

#include <stdbool.h>
#include <string.h>
#include "wifi_rx_queue.h"

	void
ath5k_ring_init (wifi_ring_t *ring)
{
	ring->head = 0;
	ring->tail = 0;
	ring->count = 0;

	ring->blocking_read = true;

	sem_init(&ring->sem, 0,0);
}

	bool
ath5k_ring_empty (wifi_ring_t *ring)
{
	return ring->count == 0;
}

	void
ath5k_ring_insert (wifi_ring_t *ring, const unsigned char *data,
		const unsigned short len, const int link_quality, const int noise, const unsigned char rate)
{
	memcpy(ring->frames[ring->head].info, data, len);
	ring->frames[ring->head].len = len;
	ring->frames[ring->head].link_quality = link_quality;
	ring->frames[ring->head].noise=noise;
	ring->frames[ring->head].rate=rate;
	ring->head = (ring->head + 1) % RX_RING_ELEMENTS;
	if (ring->count < RX_RING_ELEMENTS)
	{
		ring->count++;
	}
	else
	{
		ring->tail = (ring->tail + 1) % RX_RING_ELEMENTS;
	}
}

	void
ath5k_ring_extract (wifi_ring_t *ring, frame_t *frame)
{
	frame->len = 0;
	if (ring->count > 0)
	{
		frame->len = ring->frames[ring->tail].len;
		memcpy(frame->info, ring->frames[ring->tail].info, frame->len);
		frame->link_quality = ring->frames[ring->tail].link_quality;
		frame->noise = ring->frames[ring->tail].noise;
		frame->rate = ring->frames[ring->tail].rate;
		ring->tail = (ring->tail + 1) % RX_RING_ELEMENTS;
		ring->count--;
	}
}

