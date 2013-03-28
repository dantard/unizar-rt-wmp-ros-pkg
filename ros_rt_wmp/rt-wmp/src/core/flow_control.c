/*------------------------------------------------------------------------
 *---------------------           RT-WMP              --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: ./src/core/flow_control.c
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
#include "include/flow_control.h"
#include "include/queues.h"

static int * count;
static int * status;
static int n_nodes;

void initFlowControl(int numNodes){
	int i = 0;
	n_nodes = numNodes;
	count= (int *) MALLOC(numNodes*sizeof(int));
	status=(int *) MALLOC(numNodes*sizeof(int));
	for (i = 0; i < numNodes; i++){
		status[i]=FC_CONTINUE_SEND;
		count [i]=0;
	}
}

void freeFlowControl(void){
   FREE(count);
   FREE(status);
}

void fc_sent(int i){
	count[i] = 0;
}

int fc_get_status(int i){
	return status[i];
}

void fc_new_pap(){
	int i;
	for (i = 0 ; i< n_nodes ;i++){
		count[i]++;
	}
}

int fc_can_send(int i){
	return (count[i] > status[i]);
}

void fc_set_status(int i, int j){
	if (status[i] >= FC_CRITICAL){
		if(j == FC_DISCARDED){
			status[i]= status[i] * 4;
		}else if (j == FC_QUARTER){
			status[i]= status[i] * 100 / 75;
		} else if (j == FC_HALF){
			status[i]= status[i] * 100 / 50;
		} else if (j == FC_CONTINUE_SEND) {
			status[i]= status[i] * 100 / 25;
		}
	}else {
		status[i]=j;
	}
}

int fc_get_congestion(){
	int ret, free_pos_rx_queue = (wmp_queue_rx_get_size() - wmp_queue_rx_get_len());
#ifdef NODE_BASED
	if (free_pos_rx_queue > status.N_NODES){
		ret = FC_CONTINUE_SEND;
	} else if (free_pos_rx_queue > (status.N_NODES / 2)) {
		ret = FC_HALF;
	} else if (free_pos_rx_queue > 0){
		wmp_queue_rx_push_received(&m);
		ret = FC_QUARTER;
	} else{
		ret = FC_DISCARDED;
	}
#else
	int qs = wmp_queue_rx_get_size();
	if (free_pos_rx_queue >  qs / 2 ){
		ret = FC_CONTINUE_SEND;
	} else if (free_pos_rx_queue > (qs / 4)) {
		ret = FC_HALF;
	} else if (free_pos_rx_queue > qs / 8){
		ret = FC_QUARTER;
	} else if (free_pos_rx_queue > qs / 16){
		ret = FC_CRITICAL;
	} else if (free_pos_rx_queue > 0) {
		ret = FC_WARNING;
	} else {
		ret = FC_DISCARDED;
	}
	return ret;
#endif
}




