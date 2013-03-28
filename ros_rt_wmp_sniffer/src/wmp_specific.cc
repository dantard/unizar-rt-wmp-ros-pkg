/*------------------------------------------------------------------------
 *---------------------           WMPSNIFFER          --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: window1_glade.cc
 *  Authors: Danilo Tardioli
 *  ----------------------------------------------------------------------
 *  Copyright (C) 2000-2012, Universidad de Zaragoza, SPAIN
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
#include <math.h>
#include "misc.h"

#include "core/include/frames.h"
#include <stdio.h>
char * wmp_get_frame_tail_pointer(wmpFrame * t, int n_nodes){
	char * tkn_data_ptr =(char*)t;
	if (t->hdr.type==TOKEN){
		tkn_data_ptr+=(sizeof(Token_Hdr)+sizeof(Token)+n_nodes*n_nodes);
	}
	else if	(t->hdr.type==AUTHORIZATION){
		tkn_data_ptr+=(sizeof(Token_Hdr)+sizeof(Authorization));
	}else if	(t->hdr.type==MESSAGE){
		tkn_data_ptr+=(sizeof(Token_Hdr)+sizeof(Message)+t->msg.len);
	}else {
		fprintf(stderr,"Unknown frame type in get_tail_pointer, (s:%lu), exiting\n",t->hdr.serial);
	}
	return tkn_data_ptr;
}

char * wmp_get_frame_routing_pointer(wmpFrame * t, int n_nodes){
	char * tkn_data_ptr = (char*) t;
	if (t->hdr.type == TOKEN) {
		tkn_data_ptr += (sizeof(Token_Hdr) + sizeof(Token));
	} else if (t->hdr.type == AUTHORIZATION) {
		tkn_data_ptr += (sizeof(Token_Hdr) + sizeof(Authorization));
	} else if (t->hdr.type == MESSAGE) {
		tkn_data_ptr += (sizeof(Token_Hdr) + sizeof(Message));
	} else if (t->hdr.type == DROP_TOKEN) {
		tkn_data_ptr += (sizeof(Token_Hdr) + sizeof(Drop));
	} else {
		fprintf(stderr,
				"Unknown frame type in get_routing_pointer (s:%lu), exiting\n",
				t->hdr.serial);
	}
	return tkn_data_ptr;
}

int valid_frame(wmpFrame * p, int nbytes, int num_nodes) {
    int res = 0;
    if (nbytes < sizeof (Token_Hdr)) {
    	res +=1;
    }
    if (p->hdr.from < 0 ||
            p->hdr.to < 0 ||
            p->hdr.from >= num_nodes ||
            p->hdr.to >= num_nodes) {
    	res +=10;
    }
    if (res) {
        fprintf(stderr, "***** WARNING DISCARDING FRAME %d serial %d from %d to %d size:%d th_size:%d reason:%d\n", p->hdr.type,
               (int)p->hdr.serial, p->hdr.from, p->hdr.to, nbytes,sizeof (Token_Hdr), res);
    }
    return (!res);
}


int wmp_get_frame_total_lenght(wmpFrame * t, int n_nodes){
	int size=sizeof(Token_Hdr);
#ifdef WMP_ROUTING_tree
	size += n_nodes;
	if (t->hdr.type==TOKEN){
			size+=sizeof(Token)+2*n_nodes;
		} else if (t->hdr.type==AUTHORIZATION){
			size+=sizeof(Authorization);
		} else if (t->hdr.type==MESSAGE){
			size+=sizeof(Message)+t->msg.len;
		} else if (t->hdr.type==DROP_TOKEN){
			size+=sizeof(Drop);
		} else {
			size+=0;
		}
#else
	if (t->hdr.type==TOKEN){
		size+=sizeof(Token)+n_nodes*n_nodes;
	} else if (t->hdr.type==AUTHORIZATION){
		size+=sizeof(Authorization);
	} else if (t->hdr.type == MESSAGE) {
		if (mBitsIsSet(t->msg.type, AURA_MSG)) {
			size += sizeof(Message) + t->msg.len;
		} else {
			size += sizeof(Authorization);
		}
	} else if (t->hdr.type==DROP_TOKEN){
		size+=sizeof(Drop);
	} else {
		size+=0;
	}
#endif
#ifdef	ENABLE_BC_SUPPORT
	if (t->hdr.bc_type!=0){
		//XXX : TODO: COGLIONEEEEEEEEEE
		size+=t->hdr.bc_len;
	}
#endif
	return size;
}



char * wmp_get_message_data_pointer(wmpFrame * t){
	char * tkn_data_ptr =(char*)t;
	if (t->hdr.type!=MESSAGE) return NULL;
	tkn_data_ptr+=(sizeof(Token_Hdr)+sizeof(Message));
	return tkn_data_ptr;
}

int key(Message * p){
	int val = 0;
	char * m = (char *) p;
	for (int i=0; i < p->len ; i++){
		val += *((m+sizeof(Message))+i);
	}
	return (p->src*10000 + p->dest * 1000 + 100 * p->len + 10 * p->priority + val);
};

char * wmp_get_frame_routing_pointer_tree(wmpFrame * t, int n_nodes){
	char * tkn_data_ptr =(char*)t;
	if (t->hdr.type==TOKEN){
		tkn_data_ptr+=(sizeof(Token_Hdr)+sizeof(Token));
	}
	else if	(t->hdr.type==AUTHORIZATION){
		tkn_data_ptr+=(sizeof(Token_Hdr)+sizeof(Authorization));
	}else if	(t->hdr.type==MESSAGE){
		tkn_data_ptr+=(sizeof(Token_Hdr)+sizeof(Message));
	}else if (t->hdr.type==DROP_TOKEN) {
		tkn_data_ptr+=(sizeof(Token_Hdr)+sizeof(Drop));
	}else{
		fprintf(stderr,"Unknown frame type in get_routing_pointer (s:%lu), exiting\n",t->hdr.serial);
	}
	return tkn_data_ptr;
}
