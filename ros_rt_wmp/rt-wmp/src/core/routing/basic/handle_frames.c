/*------------------------------------------------------------------------
 *---------------------           RT-WMP              --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: ./src/core/routing/basic/handle_frames.c
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

#include "wmp_config.h"
#include "core/include/frames.h"
#include "core/include/wmp_com.h"
#include "core/interface/wmp_interface.h"

//#define WORST_LQM

int timer_initied=0;

void decode_routing_info(wmpFrame *p){

	/* Update received matrix */
	int i, j;

	char *cp;
	if (p->hdr.type == TOKEN) {
		cp = wmp_get_frame_routing_pointer(p);
		for (i = 0; i < status.N_NODES; i++) {
			for (j = 0; j < status.N_NODES; j++) {
				if (i != j) {
					lqm_set_val(i, j, *cp);
				} else {
					nstat_set_byte(i, *cp);
				}
				cp++;
			}
		}
	}
	/* Put my data in */
	for (i = 0; i< status.N_NODES; i++){
		if (i!=status.id){
			lqm_set_val(i,status.id,rssi_get_averaged_rssi(i));
		}
	}

	if (lqm_fake_is_set()){
		lqm_put_fake(lqm_get_ptr());
	}

	lqm_calculate_distances();
}

void encode_routing_info(wmpFrame * t){
	int i,j;

	if (lqm_fake_is_set()){
		lqm_put_fake(lqm_get_ptr());
	}

	char *cp = wmp_get_frame_routing_pointer(t);
	for (i = 0; i < status.N_NODES; i++) {
		for (j = 0; j < status.N_NODES; j++) {
			if (i != j) {
				(*cp) = lqm_get_val(i, j);
			} else {
				(*cp) = nstat_get_byte(i);
			}
			cp++;
		}
	}
}




