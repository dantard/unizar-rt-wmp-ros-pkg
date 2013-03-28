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

//#define WORST_LQM

int timer_initied=0;

extern void print_matrix(char* txt, char **lqm);
extern void print_vector(char*txt, char *t);
extern void tree_get_next_tree(char * cp);
extern void tree_create_lqm_from_tree(char ** lqm, char * tree);
extern void tree_create_tree_from_lqm(char * tree, char ** lqm);
extern int tree_which_best(char * t1, char * t2);

void decode_routing_info_pap(wmpFrame *p){
	int i, all_reached;
   char * cp;
	p->tkn.new_token=0;

	cp = wmp_get_frame_routing_pointer_status(p);
	for (i=0;i<status.N_NODES;i++){
		nstat_set_byte(i,cp[i]);
	}

	all_reached = 1;
	for (i = 0; i < status.N_NODES; i++) {
		if (i != status.id){
			all_reached = all_reached && nstat_isReached(i);
		}
	}

	if (all_reached){
		cp = wmp_get_frame_routing_pointer_new_tree(p);
		tree_get_next_tree(cp);
		tree_create_lqm_from_tree(lqm_get_ptr(),cp);
	} else{
		cp = wmp_get_frame_routing_pointer(p);
		tree_create_lqm_from_tree(lqm_get_ptr(),cp);
	}

	lqm_calculate_distances();
}



void encode_routing_info_pap(wmpFrame * p) {
	int i;
   char * cp_s;
   char * cp;
   char * cp_nt;
	//fprintf(stderr, "encode_pap\n");

	/* Create frame, setting reached and lqm */
	cp = wmp_get_frame_routing_pointer(p);


	print_matrix("before", lqm_get_ptr());
	tree_create_tree_from_lqm(cp, lqm_get_ptr());
	print_matrix("after", lqm_get_ptr());
	print_vector("done",cp);
	cp_s = wmp_get_frame_routing_pointer_status(p);
	for (i = 0; i < status.N_NODES; i++) {
		cp_s[i] = nstat_get_byte(i);
	}

	cp_nt = wmp_get_frame_routing_pointer_new_tree(p);

	//fprintf(stderr, "Last RECVD: %d \n", status.lastRecvdType);
	if (p->tkn.new_token && status.lastRecvdType != TOKEN) {
		//fprintf(stderr,"&&&&&&& a que hago esto %d\n",p->hdr.serial);

		memcpy(cp_nt,cp,status.N_NODES);
	}

	tree_get_next_tree(cp_nt);

	if (p->tkn.new_token) {
		//fprintf(stderr,"&&&&&&& o esto %d\n",p->hdr.serial);
		if (tree_which_best(cp,cp_nt)==2){
			memcpy(cp, cp_nt, status.N_NODES);
		}
	}
	print_vector("done2",cp);
}

void encode_routing_info_atp_mtp(wmpFrame * p){
	//fprintf(stderr,"*********** encode_atp type:%d serial:%d\n",p->hdr.type,p->hdr.serial);
	char * cp = wmp_get_frame_routing_pointer(p);
	tree_create_tree_from_lqm(cp,lqm_get_ptr());
}

void decode_routing_info_atp_mtp(wmpFrame * p){
	//fprintf(stderr,"decode atp type: %d serial:%d \n",p->hdr.type,p->hdr.serial);
	char * cp = wmp_get_frame_routing_pointer(p);
	tree_create_lqm_from_tree(lqm_get_ptr(),cp);
}

void decode_routing_info(wmpFrame * p){
	//fprintf(stderr,"Type: %d, serial:%d \n",p->hdr.type,p->hdr.serial);
	if (p->hdr.type == TOKEN){
		decode_routing_info_pap(p);
	} else if (p->hdr.type != DROP_TOKEN){
		decode_routing_info_atp_mtp(p);
	}
}

void encode_routing_info(wmpFrame * p){
	//fprintf(stderr,"encode type: %d\n",p->hdr.type);
	if (p->hdr.type == TOKEN){
		encode_routing_info_pap(p);
	} else if (p->hdr.type != DROP_TOKEN){
		encode_routing_info_atp_mtp(p);
	}
}
