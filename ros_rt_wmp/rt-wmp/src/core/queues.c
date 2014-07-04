/*------------------------------------------------------------------------
 *---------------------           RT-WMP              --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: ./src/core/queues.c
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
#include "config/compiler.h"
#include "include/queues.h"
#include "include/flow_control.h"
#include "interface/wmp_interface.h"
#include "include/queue_core.h"
#include "include/frames.h"
#include "include/task_timing.h"

queue_t wmp_queue_rx;
queue_t wmp_queue_tx;

int wmp_queue_rx_get_size() {
	return queue_get_size(&wmp_queue_rx);
}
int wmp_queue_rx_get_len() {
	return queue_rx_get_count(&wmp_queue_rx, 0);
}

void wmp_queue_tx_force_sort(int blocking) {

}

int wmp_queue_tx_inspect(void) {
	return queue_tx_inspect_head(&wmp_queue_tx, 0);
}

int wmp_queue_tx_get_len(void) {
	return queue_tx_get_count(&wmp_queue_tx);
}

int wmp_queue_tx_get_size(void) {
	return queue_get_size(&wmp_queue_tx);
}
int wmp_queue_tx_get_mpm_age(void) {
	return queue_tx_get_head_age(&wmp_queue_tx);
}

int wmp_queue_tx_get_mpm_dest(void) {
	return queue_tx_get_head_dest(&wmp_queue_tx);
}

int wmp_queue_tx_get_elem_age(int id) {
	return queue_get_elem_age(&wmp_queue_tx, id);
}

void wmp_queue_tx_drop_elem(int id){
	queue_tx_drop_elem(&wmp_queue_tx, id);
}

int wmp_queue_tx_get_elem_port(int id) {
	return queue_get_elem_port(&wmp_queue_tx, id);
}

int wmp_queue_tx_get_head_id() {
	return queue_tx_get_head_id(&wmp_queue_tx);
}

int wmp_queue_tx_get_elem_dest(int id) {
	return queue_get_elem_dest(&wmp_queue_tx, id);
}
int wmp_queue_tx_get_elem_priority(int id) {
	return queue_get_elem_priority(&wmp_queue_tx, id);
}

int wmp_queue_rx_get_elem_size(int id) {
	return queue_get_elem_size(&wmp_queue_rx, id);
}

int wmp_queue_rx_get_elem_priority(int id) {
	return queue_get_elem_priority(&wmp_queue_rx, id);
}

int wmp_queue_rx_get_elem_source(int id) {
	return queue_get_elem_source(&wmp_queue_rx, id);
}

char * wmp_queue_rx_get_elem_data(int id) {
	return queue_get_elem_data(&wmp_queue_rx, id);
}

int wmp_queue_tx_get_elem_rescheduled(int id) {
	return queue_get_elem_rescheduled(&wmp_queue_rx, id);
}

void wmp_queue_rx_set_elem_done(int id) {
	clear(&wmp_queue_rx,id);

}

int wmp_queue_tx_get_elem_burst(int id) {
	return queue_get_elem_burst(&wmp_queue_tx, id);
}

void wmp_queue_free(void) {
	queue_rx_free(&wmp_queue_rx);
	queue_tx_free(&wmp_queue_tx);
}

int wmp_queue_rx_push_part(longMsg_t * p) {
	return queue_push_part(&wmp_queue_rx, p);
}

int wmp_queue_tx_pop_part(longMsg_t ** p) {
	return queue_tx_pop_part(&wmp_queue_tx, p);
}

void wmp_queue_tx_pop_part_done(int id) {
	queue_tx_pop_part_done(&wmp_queue_tx, id);
}

int wmp_queue_tx_get_count(void) {
	return queue_tx_get_count(&wmp_queue_tx);
}

void wmp_queue_tx_drop_next(void){
	queue_tx_drop_next(&wmp_queue_tx);
}

int wmp_queue_tx_get_room(void) {
	return (queue_get_size(&wmp_queue_tx) - queue_tx_get_count(&wmp_queue_tx));
}

int wmp_queue_rx_get_room(void) {
	return queue_rx_get_room(&wmp_queue_rx);
}

int wmp_queue_tx_remove_head() {
	return queue_tx_remove_head(&wmp_queue_tx);
}
int wmp_queue_tx_reschedule(){
	return queue_tx_reschedule(&wmp_queue_tx);
}

int wmp_queue_tx_confirm(){
	return queue_tx_confirm(&wmp_queue_tx);
}

void wmp_queue_tx_done(){
	queue_tx_done(&wmp_queue_tx);
}


int wmp_queue_rx_get_count(int port){
	return queue_rx_get_count(&wmp_queue_rx, port);
}
void wmp_queue_tx_get_last_popped_info(int * age, int *port, int * priority){
	queue_tx_get_last_popped_info(&wmp_queue_tx,age,port, priority);
}
/* PUSH & POP FUNCTIONS*/

int wmpGetNumOfElementsInTXQueue(void) {
	return wmp_queue_tx_get_count();
}

int wmpGetNumOfElementsInRXQueue(int port) {
	return wmp_queue_rx_get_count(port);
}

int wmpPush(Msg *p) {
	unsigned int dest = (1 << p->dest);
	if (p->dest > ((1 << wmpGetNumOfNodes())-1)){
		//XXX:WMP_ERR(KERN_ERR "Erroreeeeee2\n");
		return 0;
	}
	task_push(p->priority);
	return queue_tx_push_data(&wmp_queue_tx, p->port, p->data, p->len, dest, p->priority);
}

void wmp_queue_init(int rx_nelems, int tx_nelems, int max_msg_size, int num_ports) {
	int max_part_size = 1500 - sizeof(Token_Hdr) - sizeof(Message) - 60;
	queue_tx_init(&wmp_queue_tx, max_msg_size, tx_nelems, max_part_size);
	queue_rx_init(&wmp_queue_rx, max_msg_size, rx_nelems, num_ports); /* 32 rx ports --> 32 semaphores!*/
	task_init();
}

unsigned int wmpPop(Msg * p) {
	char *q;
	int ts, id = queue_rx_pop_data(&wmp_queue_rx, 0, 0, &q, &p->len, &p->src,
			&p->priority);
	memcpy(p->data, q, p->len);
	ts = queue_get_elem_age(&wmp_queue_rx, id);
	queue_rx_pop_data_done(&wmp_queue_rx, id);
	return ts;
}

int wmpTimedPop(Msg * p, int timeout_ms) {
	char *q;
	int ts=-1, id = queue_rx_pop_data(&wmp_queue_rx, timeout_ms, 0, &q,
			&p->len, &p->src, &p->priority);
	if (id>=0){
		memcpy(p->data, q, p->len);
		ts = queue_get_elem_age(&wmp_queue_rx, id);
	}
	queue_rx_pop_data_done(&wmp_queue_rx, id);
	return ts;
}

int wmpNonBlockingPop(Msg * p) {
	return wmpTimedPop(p, 1);
}

void wmpAddDest(unsigned int dest, char id) {
	dest = dest | (1 << id);
}

int wmpPushData(unsigned int port, char  * p, unsigned int   size, unsigned int   dest, signed char priority){

	unsigned int filter = (1 << wmpGetNumOfNodes()) - 1;
	dest = dest & filter;

	if (dest == (1 << wmpGetNodeId())){
		return queue_rx_push_loop_data(&wmp_queue_rx, port, p, size, dest, priority, wmpGetNodeId());
	}else if (dest & (1 << wmpGetNodeId())){
		queue_rx_push_loop_data(&wmp_queue_rx, port, p, size, dest, priority, wmpGetNodeId());
		dest = dest & ~(1 << wmpGetNodeId());
		task_push(priority);
		return queue_tx_push_data(&wmp_queue_tx, port, p, size, dest, priority);
	}else{
		task_push(priority);
		return queue_tx_push_data(&wmp_queue_tx, port, p, size, dest, priority);
	}
}

int wmpPopData (unsigned int port, char ** p, unsigned int * size, unsigned char * src, signed char * priority){
	return queue_rx_pop_data(&wmp_queue_rx, 0, port, p, size, src, priority);
}

int wmpPopDataTimeout(unsigned int port, char ** p, unsigned int * size, unsigned char * src, signed char * priority, int to){
	return queue_rx_pop_data(&wmp_queue_rx, to, port, p, size, src, priority);
}

int wmpWaitData(unsigned int port, int to){
	return queue_rx_wait_data(&wmp_queue_rx, port, to);
}

void wmpPopDataDone(int id) {
	queue_rx_pop_data_done(&wmp_queue_rx, id);
}

int wmpPopDataCopy (unsigned int port, char  * p, unsigned int * size, unsigned char * src, signed char * priority){
	char * q;
	int id = wmpPopData(port,&q,size,src,priority);
	memcpy(p,q,*size);
	wmpPopDataDone(id);
	return id;
}

int wmpPopDataTimeoutCopy (unsigned int port, char  * p, unsigned int * size, unsigned char * src, signed char * priority, int to){
	char * q;
	int id = wmpPopDataTimeout(port,&q,size,src,priority, to);
	memcpy(p,q,*size);
	wmpPopDataDone(id);
	return id;
}

void wmpForceBurst(int port){
	queue_tx_force_burst(&wmp_queue_tx,port);
}

