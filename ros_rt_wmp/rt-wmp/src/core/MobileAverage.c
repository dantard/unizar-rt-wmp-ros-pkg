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
void mobile_avg_init(MobileAverage * e, int n_elements){
	e->elem=(char *) MALLOC(n_elements*sizeof(char));
	e->n_elements=n_elements;
	e->seen=0;
	e->idx=0;
	e->initialized=0;
	e->avgd_value=0; /* or 0 better */
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

void mobile_avg_reset(MobileAverage* e) {
	//XXX
	e->initialized = 0;
	e->avgd_value = 0;
};


char mobile_avg_get_averaged_value(MobileAverage * e){
	return e->avgd_value;
};

unsigned long mobile_avg_get_age(MobileAverage * e){
   return (DO_DIV64(getRawActualTimeus(),1000) - DO_DIV64(e->seen,1000)) ;
};





