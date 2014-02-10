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

//XXX:TEMP

char ** lqm_put_fake(char ** mlqm) {
   int i, j, val_ij, val_ji, found;
   char fake[16] = {0, 10, 99, 10, 10, 0, 10, 99, 99, 10, 0, 99, 10, 99, 99, 0};
   int size = wmpGetNumOfNodes();
   for (i = 0; i < size; i++) {
		for (j = 0; j < size; j++) {
			if (i==j){

			}else{
				mlqm[i][j] = fake[i*size+j];
			}
		}
   }
   return mlqm;
}


void decode_routing_info(wmpFrame *p){
	//wmp_print_lqm(lqm_get_ptr(),"before decode",wmpGetNumOfNodes());
	/*if is a TOKEN update local LQM and unset new_token*/
	int i,j;
	char *cp;
	if (p->hdr.type == TOKEN){
		cp= wmp_get_frame_routing_pointer(p);
		for (i=0;i<status.N_NODES;i++){
			for (j=0;j<status.N_NODES;j++){
				if (i!=j) {
					if (j == wmpGetNodeId() && i == p->hdr.from && (*cp) == 0){
						/* is for me */
						lqm_set_val(i,j,12);//rssi_get_averaged_rssi(i);
					}else{
						/* not try value */
						lqm_set_val(i,j,*cp);
					}
				}else {
					nstat_set_byte(i,*cp);
				}
				cp++;
			}
		}
	}
	//wmp_print_lqm(lqm_get_ptr(),"after1",wmpGetNumOfNodes());

	/*  Actualize LocalLQM */
	for (i=0;i<status.N_NODES;i++){
		if (i==status.id) continue;
		lqm_set_val(status.id,i,rssi_get_averaged_rssi(i));
	}
	//wmp_print_lqm(lqm_get_ptr(),"after2",wmpGetNumOfNodes());
#ifdef WORST_LQM
	for (j = 0; j < status.N_NODES; j++) {
		if (lqm_get_val(status.id, j) > lqm_get_val(j, status.id) && lqm_get_val(j,status.id) > 0 ) {
			lqm_set_val(status.id, j, lqm_get_val(j, status.id));
		}
	}
#endif

	lqm_put_fake(lqm_get_ptr());


	lqm_calculate_distances();
	//wmp_print_lqm(lqm_get_ptr(),"after3",wmpGetNumOfNodes());
}




void encode_routing_info(wmpFrame * t){
	int i,j;
	/* Create frame, setting reached and lqm */

#ifdef WORST_LQM
	for (i=0;i<status.N_NODES;i++){
		if (i==status.id) continue;
		lqm_set_val(status.id,i,rssi_get_averaged_rssi(i));
	}
#endif

	char *cp=wmp_get_frame_routing_pointer(t);
	for (i=0;i<status.N_NODES;i++){
		for (j=0;j<status.N_NODES;j++){
			if (i!=j) {
				(*cp) = lqm_get_val(i,j);
			}else {
				(*cp) = nstat_get_byte(i);
			}
			cp++;
		}
	}
}




