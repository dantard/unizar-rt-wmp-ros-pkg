/*------------------------------------------------------------------------
 *---------------------           RT-WMP              --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: ./src/core/include/MobileAverage.h                              
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



#ifndef RSSI_H_
#define RSSI_H_
#include "config/compiler.h"
#include "wmp_misc.h"
typedef struct{
	char *elem;
	int initialized;
	int n_elements;
	char avgd_value;
	int idx;

	int c_idx;
	char conf[255];

	int l_idx;
	char loops[255];

	int pdr;
	int rxr;

	unsigned long long seen;
	int valid;

	char node_id;
	long last_loop;

	int net_loop_id;
	int node_pulse;

} MobileAverage ;

void mobile_avg_init(MobileAverage * e, int n_elements, int node_id);
void mobile_avg_free(MobileAverage * e);
void mobile_avg_new_value(MobileAverage*e, char val);
void mobile_avg_reset(MobileAverage* e);
char mobile_avg_get_averaged_value(MobileAverage * e);
unsigned long mobile_avg_get_age(MobileAverage * e);
void mobile_avg_confiability_new_value(MobileAverage * e, char val);
void mobile_avg_new_loop(MobileAverage* e, long loop_id);

#endif /*RSSI_H_*/

