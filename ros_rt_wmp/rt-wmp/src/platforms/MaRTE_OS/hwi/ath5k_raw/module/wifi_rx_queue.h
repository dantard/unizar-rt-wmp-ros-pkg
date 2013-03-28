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
 *----------------------------------------------------------------------------
 *
 *                            'r x _ q u e u e . h'
 *
 *                                     C
 *
 *
 * File 'rx_queue.h'                                                  By Chema.
 *                                                          Jose Maria Martinez
 *                                                            <chema@gmx.net>
 * Body fuctions on 'rx_queue.c'

 *---------------------------------------------------------------------------*/

#ifndef _MARTE_WIFI_RX_QUEUE_H
#define _MARTE_WIFI_RX_QUEUE_H


/* This module implements a RING ring queue. It is intended */
/* to be a RING reception queue with a ring storage behavior*/

#include <semaphore.h>
#include <stdbool.h>
#include "ath5k_interface.h"

#define RX_RING_ELEMENTS	500

typedef struct
{
	unsigned int head, tail, count;
	frame_t frames [RX_RING_ELEMENTS];
	bool blocking_read;
	sem_t sem;
} wifi_ring_t;

/*init_ring : This function inicialices the RING queue.*/
/*            you MUST call it BEFORE any access to the queue*/
void ath5k_ring_init (wifi_ring_t *ring);

/*is_ring_empty : Checks if the queue is empty. Returns FALSE if*/
/*                thereis any frame to be read and TRUE if the  */
/*                queue is empty. */
bool ath5k_ring_empty (wifi_ring_t *ring);


/* inset_ring : Stores E in the ring buffer F. The free_pos & frames_num */
/*              variables are updated.                                   */
/*              Return 0 if OK.                                          */
/*              RING_FULL if no room in buffer.                          */
void ath5k_ring_insert (wifi_ring_t *ring, const unsigned char *data,
		const unsigned short len, const int link_quality, const int noise, const unsigned char rate);


/*read_ring : Copies in E the the first-in-frame in the buffer.*/
/*            Returns the number of rx bytes. Updates current_pos */
/*            variable.It works under interrupts inhibited. */
/*            this function is blocking y default. If it is configured*/
/*            non blocking QUEUE_EMPTY is returned when empty.*/
void ath5k_ring_extract (wifi_ring_t *ring, frame_t *frame);

#endif /*_MARTE_WIFI_RX_QUEUE_H*/
