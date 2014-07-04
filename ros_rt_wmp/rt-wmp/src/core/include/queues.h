/*------------------------------------------------------------------------
 *---------------------           RT-WMP              --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: ./src/core/include/queues.h                                     
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




#ifndef QUEUES_H_
#define QUEUES_H_

#include "config/compiler.h"
#include "core/interface/Msg.h"
#include "core/include/queue_core.h"


void wmp_queue_init(int rx_nelems, int tx_nelems, int max_msg_size, int num_ports);
void wmp_queue_free(void);

int  wmp_queue_tx_inspect(void);
int  wmp_queue_tx_get_len(void);
int wmp_queue_tx_get_size(void);

int wmp_queue_rx_get_size(void);
int wmp_queue_rx_get_len(void);
int wmp_queue_rx_push_received(Msg * p);
int wmp_queue_rx_push_part(longMsg_t * p);
int wmp_queue_rx_get_elem_size(int id);
int wmp_queue_rx_get_elem_priority(int id);
int wmp_queue_rx_get_elem_source(int id);
char * wmp_queue_rx_get_elem_data(int id);
void wmp_queue_rx_set_elem_done(int id);
int wmp_queue_rx_get_room(void);

int wmp_queue_tx_pop_part(longMsg_t ** p);
void wmp_queue_tx_pop_part_done(int id);
int wmp_queue_tx_get_count(void);
int wmp_queue_tx_get_head_age(void);
int wmp_queue_tx_get_head_dest(void);
int wmp_queue_tx_get_head_id(void);
int wmp_queue_tx_get_elem_age(int id);
int wmp_queue_tx_get_elem_dest(int id);
int wmp_queue_tx_get_elem_priority(int id);
int wmp_queue_tx_get_elem_burst(int id);
int wmp_queue_tx_reschedule(void);
int wmp_queue_tx_confirm(void);

//XXX:
void wmp_queue_tx_done(void);


int wmp_queue_tx_get_elem_port(int id);
void wmp_queue_tx_drop_elem(int id);
void wmp_queue_tx_get_last_popped_info(int * age, int *port, int * priority);
int wmp_queue_tx_get_elem_rescheduled(int id);
void wmp_queue_tx_drop_next(void);

int getNumOfElementInRXUCQueue(void);
int getNumOfElementInTXUCQueue(void);
int getNumOfFreePositionsInTXUCQueue(void);

char getHighestPriorityInTxUCQueue(void);
unsigned int getHighestPriorityAgeInTxUCQueue(void);
unsigned int popUCMsgToSend(Msg * p);



#endif /*QUEUES_H_*/



