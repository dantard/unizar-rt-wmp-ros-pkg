/*------------------------------------------------------------------------
 *---------------------           RT-WMP              --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: ./src/core/interface.c
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
#include "include/global.h"
#include "include/lqm.h"
#include "interface/wmp_interface.h"

char wmpGetNodeId(void){
	return status.id;
}

char wmpGetNumOfNodes(void){
	return status.N_NODES;
}

int wmpGetLatestLQM(char * lqm){
	int k=0,i=0,j=0;
	for (i=0;i<status.N_NODES;i++){
		for (j=0;j<status.N_NODES;j++){
			lqm[k]=lqm_get_val(i,j);
			k++;
		}
	}
	return k;
}


int wmpGetLatestDistance(char * lqm){
	int k=0,i=0,j=0;
	for (i=0;i<status.N_NODES;i++){
		for (j=0;j<status.N_NODES;j++){
			lqm[k]=lqm_get_distance(i,j);
			k++;
		}
	}
	return k;
}


char wmpGetMaxRssi(void){
	return status.max_rssi;
}

void wmpSetCpuDelay(int val){
	status.cpu_delay = val;
}

int wmpGetCpuDelay(void){
   return status.cpu_delay;
}

void wmpSetTimeout(int val){
	status.TIMEOUT = val;
}

int wmpGetTimeout(void){
   return status.TIMEOUT;
}

void wmpSetWCMult(int val){
	status.multiplier = val;
}

int wmpGetWCMult(void){
   return status.multiplier;
}

void wmpSetRate(int val){
	status.rate = val;
}

int wmpGetRate(void){
   return status.rate;
}

int wmpGetNetIT(void){
	return status.net_inactivity_timeout;
}

void wmpSetInstanceId(short iid){
	status.instance_id = iid;
}

short wmpGetInstanceId(void){
   return status.instance_id;
}


void wmp_set_levels(int w100, int w3, int w2, int w1){
	status.w100 = w100;
	status.w3 = w3;
	status.w2 = w2;
	status.w1 = w1;
}

void wmpSetMessageReschedule(int val){
	status.enable_message_reschedule = val;
}
int wmpGetMessageReschedule(void){
   return status.enable_message_reschedule;
}

void wmpSetFlowControl(int val){
	status.enable_flow_control = val;
}

int wmpGetFlowControl(void){
   return status.enable_flow_control;
}

unsigned int wmpGetSerial(void){
	return status.serial;
}

unsigned int wmpGetLoopId(void){
	return status.loop_id;
}
