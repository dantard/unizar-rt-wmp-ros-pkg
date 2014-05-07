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
#include "core/include/queue_core.h"

#ifdef USE_LONG_MESSAGE_COMPRESS
#include <zlib.h>
static Bytef * zd;
static uLongf zlen;
#endif

/*  ****  LONG MESSAGES */

static int look_tx(queue_t * q) {
	int i, maxPri = -1, maxPriId = -1;
	long long age, older_age = 0;
	for (i = 0; i < q->max_msg_num; i++) {
		if (q->longMsg[i]->done) {
			age = getRawActualTimeus() - q->longMsg[i]->ts;
			if (q->longMsg[i]->priority > maxPri || (q->longMsg[i]->priority
					== maxPri && age > older_age)) {
				maxPri = q->longMsg[i]->priority;
				maxPriId = i;
				older_age = age;
			}
		}
	}
	return maxPriId;
}

static int extend_size_if_necessary(queue_t *q, int idx, int size){
	if (size > q->longMsg[idx]->allocated_size){
		char * dp;
		WMP_ERROR(stderr, "Not enough space in element %d (%d bytes), allocating %d bytes\n",idx,q->longMsg[idx]->allocated_size,size);
		dp = MALLOC(size);
		if (dp != 0){
			FREE(q->longMsg[idx]->data);
			q->longMsg[idx]->data = dp;
			q->longMsg[idx]->allocated_size = size;
		}else{
			WMP_ERROR(stderr, "Error: Unable to allocate memory (queue_push_part), discarding message");
			q->longMsg[idx]->hash = 0;
			return 0;
		}
	}
	return 1;
}


int queue_tx_push_data(queue_t * q, unsigned int port, char * p, unsigned int size,
		unsigned int dest, signed char priority) {
	int i, nparts, must_signal = 0;
	int node_id = wmpGetNodeId();
	unsigned short ts = (unsigned short) (getRawActualTimeus() & 0x7F);

	if (q->drop_next){
		q->drop_next = 0;
		return 1;
	}
	exclusive_on(q);

	node_id = node_id << 11;
	ts = ts << 5;

	q->hash_idx++;
	if (q->hash_idx > 14) {
		q->hash_idx = 1;
	}

	nparts = size / q->message_part_size;
	if (size % q->message_part_size != 0){
		nparts++;	
	}

	int selected = -1, idx = 0;
	long long oldest;
	for (i = 0; i < q->max_msg_num; i++) {
		if (q->longMsg[i]->hash == 0) {
			selected = i;
			must_signal = 1;
			break;
		} else {
			if (q->longMsg[i]->priority <= priority) {
				if (idx == 0) {
					oldest = q->longMsg[i]->ts;
					selected = i;
				} else {
					if (q->longMsg[i]->ts < oldest) {
						oldest = q->longMsg[i]->ts;
						selected = i;
					}
				}
				idx++;
			}
		}
	}

	if (selected == -1){
		fprintf(stderr,"*** (RT-WMP) UNABLE TO FIND PLACE\n");
		exclusive_off(q);
		return 0;
	}

	q->longMsg[selected]->hash = 0;

	if (!extend_size_if_necessary(q, selected, size)) {
		exclusive_off(q);
		return 0;
	}

	q->longMsg[selected]->size = size;
	q->longMsg[selected]->dest = dest;
	q->longMsg[selected]->port = port;
	q->longMsg[selected]->priority = priority;
	q->longMsg[selected]->hash = node_id + ts +q->hash_idx;
	q->longMsg[selected]->ts = getRawActualTimeus();
	q->longMsg[selected]->msg_part_size = q->message_part_size;
	q->longMsg[selected]->num_parts = nparts;
	q->longMsg[selected]->parts_pointer = 0;
	q->longMsg[selected]->this_part_size = 0;

	/* Force Burst */
	if (q->force_burst == port) {
		q->longMsg[selected]->burst_hash = node_id + 2047;
	} else {
		q->longMsg[selected]->burst_hash = q->longMsg[selected]->hash;
	}

	memcpy(q->longMsg[selected]->data, p, size);
	q->elem++;
	q->longMsg[selected]->done = 1;

	exclusive_off(q);
	if (must_signal) {
		for (i = 0; i < nparts; i++) {
			SIGNAL(q->sem);
		}
	}
	else{
		fprintf(stderr,"*** (RT-WMP) WARNING: OVERWRITING OLD DATA IN TX QUEUE\n");
	}
	return 1;
}
void queue_tx_force_burst(queue_t * q, int port) {
	q->force_burst = port;
}

int queue_tx_get_count(queue_t * q) {
	return SEM_GET_COUNT(q->sem);
}

int queue_tx_get_port_period(queue_t * q) {
	return SEM_GET_COUNT(q->sem);
}

int queue_tx_pop_part(queue_t * q, longMsg_t ** p) {
	int id, ret = 0;

	ret = WAIT_TIMED(q->sem,5000);

	exclusive_on(q);
	if (ret == 0) {
		WMP_LM_DEBUG(stderr,"Lock pop data (TX)\n");
		id = look_tx(q);
		if (p!=0 && id >= 0 && id < q->max_msg_num) {

			/* Fill pointer */
			q->longMsg[id]->pointer = q->longMsg[id]->data + (q->longMsg[id]->parts_pointer * q->message_part_size);

			/* Fill this_part_size */
			if (q->longMsg[id]->num_parts == 1){
				q->longMsg[id]->this_part_size = q->longMsg[id]->size;
			}else{
				if (q->longMsg[id]->parts_pointer == (q->longMsg[id]->num_parts -1)){
					q->longMsg[id]->this_part_size = q->longMsg[id]->size - (q->message_part_size * q->longMsg[id]->parts_pointer) ;
				}else{
					q->longMsg[id]->this_part_size = q->message_part_size;
				}
			}

			/* Fill part_id */
			if (q->longMsg[id]->parts_pointer == 0){
				q->longMsg[id]->part_id = - q->longMsg[id]->num_parts;
			}else{
				q->longMsg[id]->part_id = q->longMsg[id]->parts_pointer;
			}
			*p = q->longMsg[id];
			q->last_popped_id = id;
		}
		return id;
	} else {
		return -1;
	}
}

int queue_tx_reschedule(queue_t * q) {
	if (q->last_popped_id >= 0 && q->last_popped_id < q->max_msg_num) {
		exclusive_on(q);
		q->longMsg[q->last_popped_id]->rescheduled = 1;
		exclusive_off(q);
		SIGNAL(q->sem);
		return 1;
	}
	return 0;
}

int queue_tx_confirm(queue_t * q) {
	if (q->last_popped_id >= 0 && q->last_popped_id < q->max_msg_num) {
		exclusive_on(q);
		q->longMsg[q->last_popped_id]->rescheduled = 0;
		q->longMsg[q->last_popped_id]->parts_pointer++;
		if (q->longMsg[q->last_popped_id]->parts_pointer
				== q->longMsg[q->last_popped_id]->num_parts) {
			clear(q, q->last_popped_id);
			q->elem--;
		}
		exclusive_off(q);
		return 1;
	}
	return 0;
}

void queue_tx_get_last_popped_info(queue_t * q, int * age, int *port, int * priority){
	if (q->last_popped_id >= 0 && q->last_popped_id < q->max_msg_num){
		*age = ((int) (getRawActualTimeus() - q->longMsg[q->last_popped_id]->ts))/1000;
		*port =  q->longMsg[q->last_popped_id]->port;
		*priority =  q->longMsg[q->last_popped_id]->priority;
	}
}

int queue_tx_inspect_head(queue_t * q, longMsg_t ** p) {
	int maxPriId = look_tx(q);
	if (maxPriId > 0) {
		if (p != 0) {
			*p = q->longMsg[maxPriId];
		}
		return (int) (getRawActualTimeus - q->longMsg[maxPriId]->ts);
	} else {
		return -1;
	}
}

int queue_tx_get_head_dest(queue_t * q){
	int maxPriId = look_tx(q);
	if (maxPriId>=0){
		return q->longMsg[maxPriId]->dest;
	}else{
		return -1;
	}
}
int queue_tx_get_head_age(queue_t * q){
	int maxPriId = look_tx(q);
	if (maxPriId>=0){
		return (int) (getRawActualTimeus() - q->longMsg[maxPriId]->ts);
	}else{
		return -1;
	}
}

void queue_tx_pop_part_done(queue_t * q, int maxPriId) {
	exclusive_off(q);
}

int queue_tx_remove_head(queue_t * q){
	int id = queue_tx_pop_part(q,0);
	queue_tx_pop_part_done(q,id);
	return id;
}

void queue_tx_drop_next(queue_t * q){
	q->drop_next = 1;
}

int queue_tx_get_head_id(queue_t * q){
	int maxPriId = look_tx(q);
	return maxPriId;
}
void queue_tx_drop_elem(queue_t * q, int id){
	exclusive_on(q);
	clear(q,id);
	if (queue_tx_get_count(q)>0){
		WAIT(q->sem);
	}
	exclusive_off(q);
}

int wmp_queue_tx_get_head_dest(void);

void queue_tx_init(queue_t * q, int max_msg_size, int max_msg_num, int msg_part_size) {

	int i, allocated = 0;
	q->max_msg_num = max_msg_num;
	q->max_msg_size = max_msg_size;
	q->message_part_size = msg_part_size;
	q->longMsg = (longMsg_t **) MALLOC(max_msg_num * sizeof(longMsg_t *));
	allocated += max_msg_num * sizeof(longMsg_t *);
	if (q->longMsg == 0) {
		WMP_ERROR(stderr,"Unable to allocate Memory (1)\n");
		return;
	}

	for (i = 0; i < max_msg_num; i++) {
		q->longMsg[i] = (longMsg_t *) MALLOC(sizeof(longMsg_t));
		allocated += sizeof(longMsg_t);
		if (q->longMsg[i] == 0) {
			WMP_ERROR(stderr,"Unable to allocate Memory (2)\n");
			return;
		}
		memset(q->longMsg[i], 0, sizeof(longMsg_t));

		q->longMsg[i]->data = MALLOC(max_msg_size);
		allocated += max_msg_size;
		if (q->longMsg[i]->data == 0) {
			WMP_ERROR(stderr,"Unable to allocate Memory (3)\n");
			return;
		}
		memset(q->longMsg[i]->data, 0, max_msg_size);
		q->longMsg[i]->allocated_size = max_msg_size;
		q->force_burst = -1;
	}

	SEM_INIT(&q->sem, 0, 0);
	MUTEX_INIT(&q->uniq_mtx);

	WMP_MSG(stderr,"*** Queues loaded, allocated %d kbytes\n", allocated/1024);
}

void queue_tx_free(queue_t * q) {
	int i;
	for (i = 0; i < q->max_msg_num; i++) {
		FREE(q->longMsg[i]->data);
		FREE(q->longMsg[i]);
	}
	FREE(q->longMsg);

#ifdef USE_LONG_MESSAGE_COMPRESS
	free(zd);
#endif

}
