/*------------------------------------------------------------------------
 *---------------------           RT-WMP              --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: ./src/core/wmp_misc.c
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
#include "include/definitions.h"
#include "include/global.h"
#include "include/wmp_misc.h"
void state_to_string(char state, char* string){
	switch (state){
		case EVALUATE_TOKEN:
				sprintf(string,"EVALUATE_TOKEN");
				break;
		case EVALUATE_AUTHORIZATION:
				sprintf(string,"EVALUATE_AUTHORIZATION");
				break;
		case EVALUATE_MESSAGE:
				sprintf(string,"EVALUATE_MESSAGE");
				break;
		case EVALUATE_DROP_TOKEN:
				sprintf(string,"EVALUATE_DROP_TOKEN");
				break;
		case EVALUATE_FOREIGN:
				sprintf(string,"EVALUATE_FOREIGN");
				break;
		case NEW_TOKEN:
				sprintf(string,"NEW_TOKEN");
				break;
		case WAIT_ACK:
				sprintf(string,"WAIT_ACK");
				break;
		case SEND_TOKEN:
				sprintf(string,"SEND_TOKEN");
				break;
		case RETRY:
				sprintf(string,"RETRY");
				break;
		case RECEIVE:
				sprintf(string,"RECEIVE");
				break;
		case INTERPRET_RECEIVED:
				sprintf(string,"INTERPRET_RECEIVED");
				break;
		case INTERPRET_ACK:
				sprintf(string,"INTERPRET_ACK");
				break;
		case SEND_MESSAGE:
				sprintf(string,"SEND_MESSAGE");
				break;
		case DECODE_ROUTING_INFO_ON_RECEIVE:
				sprintf(string,"DECODE_ROUTING_INFO_ON_RECEIVE");
				break;
		case DECODE_ROUTING_INFO_ON_WACK:
				sprintf(string,"DECODE_ROUTING_INFO_ON_WACK");
				break;
		case SEND_AUTHORIZATION:
				sprintf(string,"SEND_AUTHORIZATION");
				break;
		case TO_TOKEN_EXPIRED:
				sprintf(string,"TO_TOKEN_EXPIRED");
				break;
		case TO_AUTHORIZATION_EXPIRED:
				sprintf(string,"TO_AUTHORIZATION_EXPIRED");
				break;
		case TO_MESSAGE_EXPIRED:
				sprintf(string,"TO_MESSAGE_EXPIRED");
				break;
		case DUPLICATE_TOKEN:
				sprintf(string,"DUPLICATE_TOKEN");
				break;
		case CREATE_AUTHORIZATION:
				sprintf(string,"CREATE_AUTHORIZATION");
				break;
		case CREATE_MESSAGE:
				sprintf(string,"CREATE_MESSAGE");
				break;
		case ENQUEUE_MESSAGE:
				sprintf(string,"ENQUEUE_MESSAGE");
				break;
		case UPDATE_RSSI_ON_RECEIVE:
				sprintf(string,"UPDATE_RSSI_ON_RECEIVE");
				break;
		case UPDATE_RSSI_ON_ACK:
				sprintf(string,"UPDATE_RSSI_ON_ACK");
				break;
		default:
				sprintf(string,"UKNOWN STATE");
	}
}

unsigned char get_net_id(void){
	return status.net_id;
}

unsigned int wmpGetMTU(void){
	return status.MAXIMUM_DATA_SIZE;
}

void wmp_set_pose(double x, double y, int pow_mw){
	status.x = x;
	status.y = y;
	status.pow = pow_mw;
}
void wmp_get_pose(double * x, double * y, int * pow){
	*x = status.x;
	*y = status.y;
	*pow = status.pow;
}
void wmpSetActiveSearch(int val){
	status.active_search = val;
}

int wmpGetActiveSearch(void){
	return status.active_search;
}

void wmpSetPrimBasedRouting(int val){
	WMP_MSG(stderr,"Prim Based routing: %d\n",val);
	status.use_prim = val;
}

int wmpGetPrimBasedRouting(int val){
   return status.use_prim;
}

void wmpSetPruneBasedRouting(int val){
	WMP_MSG(stderr,"Prune Based routing: %d\n",val);
	status.use_prune = val;

}

int apply_config(Status * status, char * param, char * val) {
	if (strcmp(param, "NODE_ID") == 0) {
		status->id = atoi(val);
		return 1;
	} else if (strcmp(param, "NUM_NODES") == 0) {
		status->N_NODES = atoi(val);
		return 1;
	} else if (strcmp(param, "TIMEOUT") == 0) {
		return 1;
		status->TIMEOUT = atoi(val);
	} else if (strcmp(param, "HOLD_TIME") == 0) {
		status->hold_time = atoi(val);
		return 1;
	} else if (strcmp(param, "MAX_PER_NODE_RETRIES") == 0) {
		status->maxPerNodeRetries = atoi(val);
		return 1;
	} else if (strcmp(param, "MAX_TOTAL_RETRIES") == 0) {
		status->maxTotalRetries = atoi(val);
		return 1;
	} else if (strcmp(param, "MOBILE_AVERAGE_ELEMENTS") == 0) {
		status->mobile_average_elements = atoi(val);
		return 1;
	} else if (strcmp(param, "RX_QUEUE_ELEMENTS") == 0) {
		status->rx_queue_elements = atoi(val);
		return 1;
	} else if (strcmp(param, "TX_QUEUE_ELEMENTS") == 0) {
		status->tx_queue_elements = atoi(val);
		return 1;
	} else if (strcmp(param, "NET_ID") == 0) {
		status->net_id = atoi(val);
		return 1;
	} else if (strcmp(param, "USE_POWER_SAVE") == 0) {
		status->power_save = atoi(val);
		return 1;
	} else if (strcmp(param, "SECURE") == 0) {
		status->secure = atoi(val);
		return 1;
	}else if (strcmp(param, "USE_PRUNE") == 0) {
		status->use_prune = atoi(val);
		return 1;
	}else if (strcmp(param, "USE_PRIM") == 0) {
		status->use_prim = atoi(val);
		return 1;
	}else if (strcmp(param, "RATE") == 0) {
		status->rate = atoi(val);
		return 1;
	}else if (strcmp(param, "MAX_MSG_SIZE") == 0) {
		status->max_msg_size = atoi(val);
		return 1;
	}else if (strcmp(param, "CPU_DELAY_US") == 0) {
		status->cpu_delay = atoi(val);
		return 1;
	}
	return 0;

}

void wmpSetDefaultConfiguration(Status * s){
	s->cpu_delay=10;
	s->perHopDelay=10;
	s->id=0;
	s->maxPerNodeRetries=1;
	s->maxTotalRetries=2000;
	s->max_rssi=100;
	s->hold_time=1500;
	s->quiet=0;
	s->mobile_average_elements=50;
	s->acknow=-1;
	s->TIMEOUT=100;
	s->rx_queue_elements=50;
	s->tx_queue_elements=50;
	s->rssi_rising_factor=3;
	s->rate=60;
	s->multiplier=1;
	s->wait_ack_from=0;
	s->enable_flow_control = 0;
	s->enable_message_reschedule = 1;
	s->net_id=0;
	s->use_prim = 0;
	s->use_prune = 1;
	s->use_prob = 0;
	s->highestSerial = 0;
	s->wait_implicit_ack_from = -1;
	s->pow = -1;
	s->x = 0;
	s->y = 0;
	s->active_search = 1;
	s->lastRecvdType = -1;
	s->w100 = 30;
	s->w3 = 35;
	s->w2 = 40;
	s->w1 = 80;
	s->power_save = 1;
	s->secure = 1;
	s->max_msg_size = 65535;
	s->num_ports = 32;
	s->beluga = 0;
}


