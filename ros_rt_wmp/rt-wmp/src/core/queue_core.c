/*------------------------------------------------------------------------
 *---------------------           RT-WMP              --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: ./src/plugins/long_messages/long_messages.c
 *  Authors: Danilo Tardioli
 *  ----------------------------------------------------------------------
 *  Copyright (C) 2000-2010, Universidad de Zaragoza, SPAIN
 *
 *  Contact Addresses: Danilo Tardioli                   dantard@unizar.es
 *
 *  RT-WMP is free software; you can  redistribute it and/or  modify it
 *  under the terms of the GNU General Public License  as published by the
 *  Free Software Foundation;  either  version 2, or (at  your option) any
 *  later version.
 *
 *  RT-WMP  is distributed  in the  hope  that  it will be   useful, but
 *  WITHOUT  ANY  WARRANTY;     without  even the   implied   warranty  of
 *  MERCHANTABILITY  or  FITNESS FOR A  PARTICULAR PURPOSE.    See the GNU
 *  General Public License for more details.
 *
 *  You should have received  a  copy of  the  GNU General Public  License
 *  distributed with RT-WMP;  see file COPYING.   If not,  write to the
 *  Free Software  Foundation,  59 Temple Place  -  Suite 330,  Boston, MA
 *  02111-1307, USA.
 *
 *  As a  special exception, if you  link this  unit  with other  files to
 *  produce an   executable,   this unit  does  not  by  itself cause  the
 *  resulting executable to be covered by the  GNU General Public License.
 *  This exception does  not however invalidate  any other reasons why the
 *  executable file might be covered by the GNU Public License.
 *
 *----------------------------------------------------------------------*/
#include <stdarg.h>
#include "config/compiler.h"
#include "core/interface/wmp_interface.h"
#include "core/include/global.h"
#include "core/include/frames.h"
#include "include/queue_core.h"
#include "core/include/queues.h"
#include "core/include/wmp_misc.h"

#ifdef USE_LONG_MESSAGE_COMPRESS
#include <zlib.h>
static Bytef * zd;
static uLongf zlen;
#endif

/*  ****  LONG MESSAGES */

void clear(queue_t * q, int idx){
	if (idx >= 0 && idx < q->max_msg_num) {
		q->longMsg[idx]->done = 0;
		q->longMsg[idx]->received_parts = 0;
		q->longMsg[idx]->parts_pointer = 0;
		q->longMsg[idx]->part_id = 0;
		q->longMsg[idx]->num_parts = 0;
		q->longMsg[idx]->done = 0;
		q->longMsg[idx]->size = 0;
		q->longMsg[idx]->burst_hash = 0;
		q->longMsg[idx]->rescheduled = 0;
		q->longMsg[idx]->priority = -1;
		memset(q->longMsg[idx]->received_part,0,q->longMsg[idx]->max_message_parts);
		q->longMsg[idx]->hash = 0;
		q->longMsg[idx]->parts_sent = 0;

	}
}

/* Interface */
void inline exclusive_on(queue_t * q){
	MUTEX_WAIT(q->uniq_mtx);
}

void inline exclusive_off(queue_t * q){
	MUTEX_SIGNAL(q->uniq_mtx);
}

int queue_get_size(queue_t * q){
	return q->max_msg_num;
}
unsigned int queue_get_elem_port(queue_t * q, int id){
	if (id >=0 && id < q->max_msg_size){
		return q->longMsg[id]->port;
	}else{
		return -1;
	}
}
int queue_get_elem_age(queue_t * q, int id){
	if (id >=0 && id < q->max_msg_size){
		return ((int) (getRawActualTimeus() - q->longMsg[id]->ts))/1000;
	}else{
		return -1;
	}
}
int queue_get_elem_dest(queue_t * q, int id){
	if (id >=0 && id < q->max_msg_size){
		return (int) q->longMsg[id]->dest;
	}else{
		return -1;
	}
}

int queue_get_elem_burst(queue_t * q, int id) {
	int i, count = 0;
	if (id >= 0 && id < q->max_msg_size){
		unsigned short hash = q->longMsg[id]->burst_hash;
		for (i = 0; i < q->max_msg_num; i++) {
			if (hash == q->longMsg[i]->burst_hash) {
				count+=q->longMsg[i]->num_parts - q->longMsg[i]->parts_pointer;
				//printk(KERN_ERR "id:%d hash:%d num_parts%d parts_point:%d", id, hash,q->longMsg[i]->num_parts, q->longMsg[i]->parts_pointer);
			}
		}
	}
	return count;
}

int queue_get_elem_source(queue_t * q, int id){
	if (id >=0 && id < q->max_msg_size){
		return (int) q->longMsg[id]->src;
	}else{
		return -1;
	}
}
char *  queue_get_elem_data(queue_t * q, int id){
	if (id >=0 && id < q->max_msg_size){
		return q->longMsg[id]->data;
	}else{
		return 0;
	}
}

int queue_get_elem_priority(queue_t * q, int id){
	if (id >=0 && id < q->max_msg_size){
		return (int) q->longMsg[id]->priority;
	}else{
		return -1;
	}
}
int queue_get_elem_size(queue_t * q, int id){
	if (id >=0 && id < q->max_msg_size){
		return (int) q->longMsg[id]->size;
	}else{
		return -1;
	}
}

int queue_get_elem_rescheduled(queue_t * q, int id){
	if (id >=0 && id < q->max_msg_size){
		return q->longMsg[id]->rescheduled;
	}else{
		return -1;
	}
}
