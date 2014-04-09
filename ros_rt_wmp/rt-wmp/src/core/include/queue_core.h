/*------------------------------------------------------------------------
 *---------------------           RT-WMP              --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: ./src/plugins/long_messages/long_messages.h
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

#ifndef LONG_MESSAGES_H_
#define LONG_MESSAGES_H_
//#include "compiler.h"

//#define USE_LONG_MESSAGE_COMPRESS

#define WMP_LM_DEBUG(output,...)
//fprintf(output,__VA_ARGS__)//printk(KERN_ERR __VA_ARGS__)

typedef struct {
	unsigned char src;
	unsigned int dest;
	signed char priority;
	unsigned int size;
	unsigned int port;
	long long ts;
	unsigned int type;
	int done;
	int msg_part_size;
	unsigned short hash;
	unsigned short burst_hash;
	short num_parts, part_id;
	unsigned short received_parts;
	char * received_part;
	unsigned short max_message_parts;
	int allocated_size;
	int rescheduled;
	int this_part_size;
	int parts_pointer;
	signed char path[32];
	char * pointer;
	char * data;
} longMsg_t;

typedef struct {
	longMsg_t ** longMsg;
	//longMsg_t lastPopped;
	char * received_part;
	int force_burst;
	int max_msg_size, hash_idx;
	int max_msg_num;
	int message_part_size;
	int num_ports;
	int drop_next;
	int last_popped_id;
	MUTEX mtx, **mtxs, uniq_mtx;
	SEM_T ** sems, sem;
	int elem;
} queue_t;

void clear(queue_t * q, int idx);
void exclusive_on(queue_t * q);
void exclusive_off(queue_t * q);
int queue_get_size(queue_t * q);
int queue_get_elem_age(queue_t * q, int id);
int queue_get_elem_dest(queue_t * q, int id);
int queue_get_elem_priority(queue_t * q, int id);
int queue_get_elem_size(queue_t * q, int id);
int queue_get_elem_source(queue_t * q, int id);
char * queue_get_elem_data(queue_t * q, int id);
int queue_get_elem_burst(queue_t * q, int id);
unsigned int queue_get_elem_port(queue_t * q, int id);
int queue_get_elem_rescheduled(queue_t * q, int id);

/* RX Queue */
int queue_push_part(queue_t * q,  longMsg_t * m);
int queue_rx_get_mpm_size(queue_t * q, int port);
int queue_rx_pop_data(queue_t * q, unsigned int delay, unsigned int port, char ** data, unsigned int * size, unsigned char * src, signed char * priority) ;
void queue_rx_pop_data_done(queue_t * q, int maxPriId);
void queue_rx_init(queue_t * q, int max_msg_size, int max_msg_num, int num_ports);
int queue_rx_get_count(queue_t * q, int port);
void queue_rx_free(queue_t * q);
int queue_rx_wait_data(queue_t * q, int port, int delay);
int queue_rx_push_loop_data(queue_t * q, unsigned int port, char * p, unsigned int size, unsigned int dest, signed char priority, unsigned char src);
int queue_rx_get_room(queue_t * q);


/* TX Queue */
int queue_tx_remove_head(queue_t * q);
void queue_tx_init(queue_t * q, int max_msg_size, int max_msg_num, int msg_part_size);
int queue_tx_push_data(queue_t * q, unsigned int port, char * p, unsigned int size, unsigned int dest, signed char priority) ;
int queue_tx_pop_part(queue_t * q, longMsg_t ** p);
void queue_tx_pop_part_done(queue_t * q, int maxPriId);
int queue_tx_get_count(queue_t * q);
int queue_tx_inspect_head(queue_t * q, longMsg_t ** p);
int queue_tx_get_head_dest(queue_t * q);
int queue_tx_get_head_age(queue_t * q);
void queue_tx_free(queue_t * q);
int queue_tx_get_head_id(queue_t * q);
int queue_tx_reschedule(queue_t * q);
int queue_tx_confirm(queue_t * q);
void queue_tx_drop_elem(queue_t * q, int id);
void queue_tx_get_last_popped_info(queue_t * q, int * age, int *port, int * priority);
void queue_tx_drop_next(queue_t * q);
void queue_tx_force_burst(queue_t * q, int port);

#endif /* LONG_MESSAGES_H_ */
