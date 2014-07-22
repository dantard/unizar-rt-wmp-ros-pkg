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

unsigned char CONFIABILITY_WINDOW;
unsigned short LOOP_WINDOW;

void mobile_avg_free(MobileAverage * e) {
	FREE(e->elem);
}

void mobile_avg_init(MobileAverage * e, int n_elements, int node_id) {
	int i;

	CONFIABILITY_WINDOW = 00;
	LOOP_WINDOW = 50;

	e->elem = (char *) MALLOC(n_elements*sizeof(char));
	e->n_elements = n_elements;
	e->seen = 0;
	e->idx = 0;
	e->initialized = 0;
	e->avgd_value = 0; /* or 0 better */
	e->node_id = node_id;

	for (i = 0; i < LOOP_WINDOW; i++) {
		e->loops[i] = 1;
	}
	e->l_idx = 0;
}

void compute_average(MobileAverage*e) {
	int avg = 0, i;
	for (i = 0; i < e->n_elements; i++) {
		avg += (e->elem[i]);
	}
	avg = avg / e->n_elements;
	e->avgd_value = (char) avg;
}

void mobile_avg_new_value(MobileAverage*e, char val) {
	int i;
	e->seen = getRawActualTimeus();
	if (!e->initialized) {
		if (status.rssi_rising_factor <= 0) {
			status.rssi_rising_factor = 1;
		}

		memset(e->elem, 0, e->n_elements);

		int end = (e->n_elements / status.rssi_rising_factor);
		for (i = 0; i < end; i++) {
			e->elem[i] = val;
			e->idx++;
		}
		compute_average(e);
		e->initialized = 1;
	} else {
		e->idx++;
		if (e->idx >= e->n_elements) {
			e->idx = 0;
		}
		e->elem[e->idx] = val;
		compute_average(e);
	}
}

void mobile_avg_reset(MobileAverage* e) {
	e->initialized = 0;
	e->avgd_value = 0;
}

char mobile_avg_get_averaged_value(MobileAverage * e) {
	if (status.use_pdr){
		return (char) e->rxr;
	}

	int val = e->avgd_value;
	if (val > status.prob_99_perc_rssi_min) {
		val = 99;
	} else {
		val = val * 99 / status.prob_99_perc_rssi_min;
		val = val > 99 ? 99 : val;
	}

	return (char) val;
}

unsigned long mobile_avg_get_age(MobileAverage * e) {
	return (DO_DIV64(getRawActualTimeus(),1000) - DO_DIV64(e->seen,1000));
}

/* LOOPS */
void mobile_avg_compute(MobileAverage * e) {
	int i, sum = 0;
	for (i = 0; i < LOOP_WINDOW; i++) {
		sum += e->loops[i];
	}
	e->rxr = sum * 100 / LOOP_WINDOW >= 0 ? sum * 100 / LOOP_WINDOW : 0;
	e->rxr = e->rxr > 99 ? 99 : e->rxr;
}

void mobile_avg_new_loop_tick(MobileAverage* e, long loop_id) {
	if (loop_id != e->net_loop_id) {
		e->l_idx++;
		e->l_idx = e->l_idx < LOOP_WINDOW ? e->l_idx : 0;
		e->loops[e->l_idx] = 0;
		mobile_avg_compute(e);
		e->net_loop_id = loop_id;
	}
}

void mobile_avg_new_loop(MobileAverage* e, long loop_id) {
	e->loops[e->l_idx] = 1;
	mobile_avg_compute(e);
}

/* CONFIABILITY*/
void mobile_avg_confiability_new_value(MobileAverage * e, int val) {
	if (val < 0) {
		int i;
		for (i = 0; i < LOOP_WINDOW; i++) {
			if (e->loops[i] == 1) {
				e->loops[i] = 0;
				break;
			}
		}
		mobile_avg_compute(e);
	}
}
