/*------------------------------------------------------------------------
 *---------------------           RT-WMP              --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: ./src/core/MobileAverage.c
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

#include "include/MobileAverage.h"
#include "include/definitions.h"
#include "config/compiler.h"

extern Status status;

void mobile_avg_free(MobileAverage * e){
	FREE(e->elem);
}
void mobile_avg_init(MobileAverage * e, int n_elements, int node_id){
	int i;
	e->elem=(char *) MALLOC(n_elements*sizeof(char));
	e->n_elements=n_elements;
	e->seen=0;
	e->idx=0;
	e->initialized=0;
	e->avgd_value=0; /* or 0 better */
	e->node_id = node_id;
	for (i = 0; i< 50; i++){
		e->conf[i] = 1;
	}
	e->c_idx = 0;
	e->consecutive_loops = 0;
};

void mobile_avg_new_value(MobileAverage*e, char val){
	int i;
	int avg = 0;
	e->seen=getRawActualTimeus();
	if (!e->initialized){
		if (status.rssi_rising_factor <=0){
			status.rssi_rising_factor=1;
		}

		int end = (e->n_elements/status.rssi_rising_factor);
		for (i=0 ; i < end ; i++){
			e->elem[i]=val;
			e->idx++;
		}
		e->avgd_value=val;
		e->initialized=1;
	} else {
		e->idx++;
		if (e->idx>=e->n_elements) e->idx=0;
		e->elem[e->idx]=val;
		for (i=0;i<e->n_elements;i++){
			avg+=(e->elem[i]);
		}
		avg=avg/e->n_elements;
		e->avgd_value= (char) avg;
	}
};


void mobile_avg_new_loop(MobileAverage* e, long loop_id) {
	if (e->node_id == 2 && wmpGetNodeId()==3){
		fprintf(stderr,"id:%d loop_id: %d last_loop: %d consecutive_loops: %d\n", e->node_id, loop_id, e->last_loop, e->consecutive_loops);
	}

	if (mobile_avg_confiability_get(e) == 0 && ((loop_id - e->last_loop) == 0 || (loop_id - e->last_loop == 1))){
		e->consecutive_loops ++;
	}else{
		e->consecutive_loops = 0;
	}
	e->last_loop = loop_id;
	if (e->consecutive_loops == 500){
		mobile_avg_confiability_reset(e);
	}
}

void mobile_avg_reset(MobileAverage* e) {
	//XXX
	e->initialized = 0;
	e->avgd_value = 0;
	//mobile_avg_confiability_reset(e);
	//e->consecutive_loops = 0;
};


char mobile_avg_get_averaged_value(MobileAverage * e){

	int val = e->avgd_value * mobile_avg_confiability_get(e) / 100;
//	if (val == 0 && e->avgd_value > 0){
//		val = 1;
//	}
	if (mobile_avg_confiability_get(e) < 95){
		fprintf(stderr,"Node %d: e->avg is %d, conf is %d val is %d\n",e->node_id,e->avgd_value, mobile_avg_confiability_get(e), val);
	}
	return (char) val;
};

unsigned long mobile_avg_get_age(MobileAverage * e){
   return (DO_DIV64(getRawActualTimeus(),1000) - DO_DIV64(e->seen,1000)) ;
};


/* CONFIABILITY*/

void mobile_avg_confiability_reset(MobileAverage * e){
	int i;
	for (i = 0; i< 50; i++){
		e->conf[i] = 1;
	}
}

int mobile_avg_confiability_get(MobileAverage * e){
	int i, sum = 0;
	for (i = 0; i< 50; i++){
			sum+= e->conf[i];
	}
	return sum*2;
}

void mobile_avg_confiability_new_value(MobileAverage * e, char val){
	if (val == 0){
		e->consecutive_loops = 0;
	}
	e->conf[e->c_idx] = val;
	e->c_idx++;
	e->c_idx = e->c_idx<50?e->c_idx:0;
	//fprintf(stderr,"Node %d: is %d\n", e->node_id, mobile_avg_confiability_get(e));
}



