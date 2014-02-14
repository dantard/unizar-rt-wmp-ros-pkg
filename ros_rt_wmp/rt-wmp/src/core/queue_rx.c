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

/* Extends queue message capacity for element idx :: warning: not real-time behavior
 * It is intended to allow the reception of big messages without bothering the user
 * with defining the maximum size. The same happens with the vector received_part that
 * is used to allow out-of-order arrival of the messages
 * */

typedef struct {
	unsigned short hash;
	short part_id;
} memory_t;

static memory_t memory[10];
static int memory_id = 0;
static int hash_exists(unsigned short hash, short part_id) {
	int i;
	for (i = 0; i < 10; i++) {
		if (hash == memory[i].hash && part_id == memory[i].part_id) {
			return 1;
		}
	}
	memory[memory_id].hash = hash;
	memory[memory_id].part_id = part_id;
	memory_id++;
	if (memory_id == 10) {
		memory_id = 0;
	}
	return 0;
}


static int extend_size_if_necessary(queue_t *q, int idx){
	int total_msg_size, msg_size_till_this_part, size;

	total_msg_size = q->longMsg[idx]->num_parts * q->longMsg[idx]->msg_part_size;
	msg_size_till_this_part = q->longMsg[idx]->part_id * q->longMsg[idx]->msg_part_size;
	size = total_msg_size > msg_size_till_this_part ? total_msg_size : msg_size_till_this_part;

	if (size > q->longMsg[idx]->allocated_size){
		char * dp;
		WMP_ERROR(stderr, "Not enough space in element %d (%d bytes), allocating %d bytes\n",idx,q->longMsg[idx]->allocated_size,total_msg_size);
		dp = MALLOC(size);
		if (dp != 0){
			memcpy(dp, q->longMsg[idx]->data, q->longMsg[idx]->allocated_size);
			FREE(q->longMsg[idx]->data);
			q->longMsg[idx]->data = dp;
			q->longMsg[idx]->allocated_size = size;
		}else{
			WMP_ERROR(stderr, "Error: Unable to allocate memory (queue_push_part), discarding message");
			q->longMsg[idx]->hash = 0;
			return 0;
		}
	}

	size = q->longMsg[idx]->num_parts > q->longMsg[idx]->part_id ? q->longMsg[idx]->num_parts : q->longMsg[idx]->part_id;
	if (size > q->longMsg[idx]->max_message_parts){
		char * dp;
		WMP_ERROR(stderr, "max_message_parts is too small %d (%d bytes), allocating %d bytes\n",idx,q->longMsg[idx]->max_message_parts,size);
		dp = MALLOC(size);
		if (dp != 0){
			memcpy(dp, q->longMsg[idx]->received_part, q->longMsg[idx]->max_message_parts);
			FREE(q->longMsg[idx]->received_part);
			q->longMsg[idx]->received_part = dp;
			q->longMsg[idx]->max_message_parts = size;
		}else{
			WMP_ERROR(stderr, "Error: Unable to allocate memory for extend message_parts (queue_push_part), discarding message");
			q->longMsg[idx]->hash = 0;
			return 0;
		}
	}
	return 1;
}


int queue_push_part(queue_t * q,  longMsg_t * m) {
	int i, idx = -1,  must_signal = 0, cnt=0;
	exclusive_on(q);
	if (m->port < 0 || m->port >= q->num_ports){
		WMP_ERROR(stderr, "*** WARNING: Discarded message-part due to incorrect port number (port: %d)\n", m->port);
		exclusive_off(q);
		return 0;
	}
	if (hash_exists(m->hash, m->part_id)){
		static int dropped = 0;
		dropped++;
		if (dropped % 10 == 0){
			WMP_ERROR(stderr, "*** WARNING: Discarded 10 message-parts due existent hash (hash: %d)\n",m->hash);
		}
		exclusive_off(q);
		return 0;
	}

	for (i = 0; i < q->max_msg_num; i++) {
		/* Discards unfinished old messages and take the place */
		if (q->longMsg[i]->hash != 0 && !q->longMsg[i]->done) {
			long long age = getRawActualTimeus() - q->longMsg[i]->ts;
			if (age > ((long long) 5000000)) {
				cnt++;
				clear(q, i);
			}
		}
	}
	if (cnt > 0){
		WMP_ERROR(stderr, "Discarding old message-part (count: %d)\n", cnt);
	}

	for (i = 0; i < q->max_msg_num; i++) {
		/* Try to find an already existing part */
		if (q->longMsg[i]->hash == m->hash) {
			idx = i;
			break;
		}
	}

	if (idx < 0) {
		for (i = 0; i < q->max_msg_num; i++) {
			/* Try to find a free place */
			if (q->longMsg[i]->hash == 0) {
				idx = i;
				break;
			}
		}
	}

//	if (idx < 0) {
//		for (i = 0; i < q->max_msg_num; i++) {
//			/* Discards unfinished old messages and take the place */
//			if (q->longMsg[i]->hash != 0 && !q->longMsg[i]->done) {
//				long long age = getRawActualTimeus() - q->longMsg[i]->ts;
//				if (age > ((long long) 5000000)) {
//					idx = i;
//					WMP_ERROR(stderr,"Warning: Discard uncompleted message pos:%d "
//							"hash:%d "
//							"num_parts:%d "
//							"received_parts:%d "
//							"size:%d \n", idx, q->longMsg[i]->hash, q->longMsg[i]->num_parts,q->longMsg[i]->received_parts,  q->longMsg[i]->size);
//					clear(q, i);
//
//					//break;
//				}
//			}
//		}
//	}

//  Warning: to discard you must take into account the situation of the semaphore:
//  If you discard a message you have to take into account if it was complete
//  or not and if the message had already signaled the semaphore

	if (idx >= 0) {
		int done;

		if (m->part_id < 0) {
			q->longMsg[idx]->num_parts = -m->part_id;
			m->part_id = 0;
		}

		q->longMsg[idx]->received_part[m->part_id] = 1;
		q->longMsg[idx]->src = m->src;
		q->longMsg[idx]->msg_part_size = m->msg_part_size;
		q->longMsg[idx]->size += m->size;
		q->longMsg[idx]->received_parts++;
		q->longMsg[idx]->hash = m->hash;
		q->longMsg[idx]->port = m->port;
		q->longMsg[idx]->priority = m->priority;
		q->longMsg[idx]->ts = getRawActualTimeus();

		if (!extend_size_if_necessary(q,idx)){
			exclusive_off(q);
			return 0;
		}

		memcpy(q->longMsg[idx]->data + (m->part_id * q->longMsg[idx]->msg_part_size), m->data, m->size);

		done = (q->longMsg[idx]->num_parts > 0);
		for (i=0;i<q->longMsg[idx]->num_parts;i++){
			done = done & q->longMsg[idx]->received_part[i];
		}

		if (done) {
			WMP_LM_DEBUG(stderr, "DONE hash: %d size:%d port %d \n",
					q->longMsg[idx]->hash, q->longMsg[idx]->size, q->longMsg[idx]->port);
			q->longMsg[idx]->done = 1;
			must_signal = 1;
		}
	} else {
		WMP_ERROR(stderr, "*** (RT-WMP) NO SPACE to receive new message (queued elements:%d)\n", queue_rx_get_count(q,m->port));
		exclusive_off(q);
		return -1;
	}

	exclusive_off(q);

	if (must_signal){
		SIGNAL(*q->sems[m->port]);
	}
	return 0;
}

int queue_rx_push_loop_data(queue_t * q, unsigned int port, char * p,
		unsigned int size, unsigned int dest, signed char priority, unsigned char src) {
	int i, idx =-1;
	exclusive_on(q);

	if (port < 0 || port >= q->num_ports) {
		WMP_ERROR(stderr,"Discarding loop data due to incorrect port number (%d) @ loop\n",port);
		exclusive_off(q);
		return 0;
	}

	for (i = 0; i < q->max_msg_num; i++) {
		if (q->longMsg[i]->hash == 0) {
			idx = i;
		}
	}

	if (idx >= 0) {
		q->longMsg[idx]->hash = -1;
		q->longMsg[idx]->size = size;
		q->longMsg[idx]->dest = dest;
		q->longMsg[idx]->src = src;
		q->longMsg[idx]->port = port;
		q->longMsg[idx]->ts = getRawActualTimeus();
		q->longMsg[idx]->num_parts = 1;
		q->longMsg[idx]->msg_part_size = size;
		q->longMsg[idx]->priority = priority;

		if (!extend_size_if_necessary(q,idx)){
			exclusive_off(q);
			return 0;
		}

		memcpy(q->longMsg[idx]->data, p, size);
		q->longMsg[idx]->done = 1;
		exclusive_off(q);
		SIGNAL(*q->sems[port]);
		return 1;
	} else {
		exclusive_off(q);
		return 0;
	}
}

static int look_rx(queue_t * q, int port) {
	int i, maxPri = -1, maxPriId = -1;
	long long age, older_age = 0;
	for (i = 0; i < q->max_msg_num; i++) {
		if (q->longMsg[i]->done && q->longMsg[i]->port == port) {
			age = getRawActualTimeus() - q->longMsg[i]->ts;
			if (q->longMsg[i]->priority > maxPri || (q->longMsg[i]->priority == maxPri && age > older_age)) {
				maxPri = q->longMsg[i]->priority;
				maxPriId = i;
				older_age = age;
			}
		}
	}
	return maxPriId;
}

/* Interface */

int queue_rx_get_mpm_size(queue_t * q, int port) {
	int i, maxPri = 0, maxPriId = -1;

	if (port >= q->num_ports || port < 0 ){
		WMP_ERROR(stderr," *** (RT-WMP) GetDataSize port number out of range, discarding");
		return -1;
	}

	for (i = 0; i < q->max_msg_num; i++) {
		if (q->longMsg[i]->done && q->longMsg[i]->port == port) {
			if (q->longMsg[i]->priority > maxPri) {
				maxPri = q->longMsg[i]->priority;
				maxPriId = i;
			}
		}
	}
	if (maxPriId != -1) {
		return (int) q->longMsg[maxPriId]->size;
	} else {
		return -1;
	}
}
int queue_rx_wait_data(queue_t * q, int port, int delay){
	int ret;
	if (port >= q->num_ports || port < 0) {
		WMP_ERROR(stderr,
				" *** (RT-WMP) Pop port number (%d) out of range (%d), discarding\n", port, q->num_ports);
		return -1;
	}
	if (delay == 0) {
		ret = WAIT(*q->sems[port]);
	} else if (delay > 0){
		ret = WAIT_TIMED(*q->sems[port],delay);
	}else{
		if (queue_rx_get_count(q,port)>0){
			ret = 0;
		}else{
			ret = -1;
		}
	}
	if (ret == 0){
		int id;
		exclusive_on(q);
		id = look_rx(q, port);
		exclusive_off(q);
		return id;
	}else{
		return -1;
	}
}

int queue_rx_pop_data(queue_t * q, unsigned int delay, unsigned int port, char ** data, unsigned int * size,
		unsigned char * src, signed char * priority) {
	int id, ret = 0;
	if (port >= q->num_ports || port < 0) {
		WMP_ERROR(stderr,
				" *** (RT-WMP) Pop port number (%d) out of range (%d), discarding\n", port, q->num_ports);
		exclusive_on(q);

		return -1;
	}

	if (delay == 0) {
		ret = WAIT(*q->sems[port]);
	} else if (delay >0){
		ret = WAIT_TIMED(*q->sems[port],delay);
	}else{
		ret = 0;
	}

	exclusive_on(q);

	if (ret == 0) {
		WMP_LM_DEBUG(stderr,"Lock pop data RX\n");
		id = look_rx(q, port);
		if (id >= 0 && id < q->max_msg_num) {
			if (data != 0)     (*data)     = q->longMsg[id]->data;
			if (src != 0)     (*src)      = q->longMsg[id]->src;
			if (priority !=0) (*priority) = q->longMsg[id]->priority;
			if (size != 0)    (*size)     = q->longMsg[id]->size;
		}
		return id;
	}else{
		return -1;
	}
}

void queue_rx_pop_data_done(queue_t * q, int maxPriId) {
	if (maxPriId >= 0 && maxPriId < q->max_msg_num){
		clear(q,maxPriId);
		WMP_LM_DEBUG(stderr,"Unlock pop done RX\n");
	}
	exclusive_off(q);
}

void queue_rx_init(queue_t * q, int max_msg_size, int max_msg_num, int num_ports) {

	int i, allocated = 0;
	q->max_msg_num=max_msg_num;
	q->num_ports=num_ports;
	q->max_msg_size = max_msg_size;
	q->longMsg = (longMsg_t **) MALLOC(max_msg_num * sizeof(longMsg_t *));
	allocated+=max_msg_num * sizeof(longMsg_t *);
	if (q->longMsg == 0) {
		WMP_ERROR(stderr,"Unable to allocate Memory (1)\n");
		return;
	}

	for (i = 0; i < max_msg_num; i++) {

		q->longMsg[i] = (longMsg_t *) MALLOC(sizeof(longMsg_t));
		allocated+=sizeof(longMsg_t);
		if (q->longMsg[i] == 0) {
			WMP_ERROR(stderr,"Unable to allocate Memory (2)\n");
			return;
		}
		memset(q->longMsg[i], 0, sizeof(longMsg_t));

		q->longMsg[i]->data = MALLOC(max_msg_size);
		q->longMsg[i]->allocated_size = max_msg_size;

		allocated+=max_msg_size;
		if (q->longMsg[i]->data == 0) {
			WMP_ERROR(stderr,"Unable to allocate Memory (3)\n");
			return;
		}
		memset(q->longMsg[i]->data, 0, max_msg_size);

		q->longMsg[i]->max_message_parts = ((max_msg_size/512)+1);
		q->longMsg[i]->received_part = (char *) MALLOC(q->longMsg[i]->max_message_parts);
		allocated+=q->longMsg[i]->max_message_parts;
		if (q->longMsg[i]->received_part == 0) {
			WMP_ERROR(stderr,"Unable to allocate Memory (4)\n");
			return;
		}
		memset(q->longMsg[i]->received_part, 0, q->longMsg[i]->max_message_parts);

	}

	q->sems = (SEM_T **) MALLOC(num_ports * sizeof(SEM_T *));
	q->mtxs = (MUTEX **) MALLOC(num_ports * sizeof(MUTEX *));
	allocated+=num_ports * (sizeof(SEM_T *) +  sizeof(MUTEX *));
	if (q->sems == 0 || q->mtxs == 0) {
		WMP_ERROR(stderr,"Unable to allocate Memory (5)\n");
		return;
	}

	for (i = 0; i < num_ports; i++) {
		q->sems[i] = (SEM_T *) MALLOC(sizeof(SEM_T));
		q->mtxs[i] = (MUTEX *) MALLOC(sizeof(MUTEX));
		allocated+=sizeof(SEM_T) + sizeof(MUTEX);
		if (q->sems[i] == 0 || q->mtxs[i] == 0) {
			WMP_ERROR(stderr,"Unable to allocate Memory (6)\n");
			return;
		}
		SEM_INIT(q->sems[i], 0, 0);
		MUTEX_INIT(q->mtxs[i]);
	}
	MUTEX_INIT(&q->mtx);
	MUTEX_INIT(&q->uniq_mtx);

	WMP_MSG(stderr,"*** Queues loaded, allocated %d kbytes\n", allocated/1024);
}

void queue_rx_free(queue_t * q) {
	int i;
        for (i = 0; i < q->max_msg_num; i++) {
			FREE(q->longMsg[i]->data);
			FREE(q->longMsg[i]->received_part);
			FREE(q->longMsg[i]);
        }

        FREE(q->longMsg);
        for (i = 0; i < q->num_ports; i++) {
                FREE(q->sems[i]);
                FREE(q->mtxs[i]);
        }
        FREE(q->sems);
        FREE(q->mtxs);
}

int queue_rx_get_count(queue_t * q, int port) {
	if (port >= q->num_ports || port < 0) {
		WMP_ERROR(stderr,"Warning, asking queue_rx_get_count on port %d while only %d\n", port, q->num_ports);
		return 0;
	} else {
		return SEM_GET_COUNT(*q->sems[port]);
	}
}
int queue_rx_get_room(queue_t * q){
	int i, cnt = 0;
	exclusive_on(q);
    for (i = 0; i < q->max_msg_num; i++) {
    	if (q->longMsg[i]->hash == 0){
    		cnt++;
    	}
    }
    exclusive_off(q);
    return cnt;
}
