/*------------------------------------------------------------------------
 *---------------------           RT-WMP              --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: ./src/core/routing/basic/frames.c
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

#include "core/include/global.h"
#include "core/include/frames.h"
#include "config/compiler.h"

#define TOKEN_ROUTING_INFO_SIZE status.N_NODES*status.N_NODES
#define AUTH_ROUTING_INFO_SIZE 0
#define MSG_ROUTING_INFO_SIZE  0

char * wmp_get_frame_tail_pointer(wmpFrame * t){

	char * tkn_data_ptr =(char*)t;
	if (t->hdr.type==TOKEN){
		tkn_data_ptr+=(sizeof(Token_Hdr)+sizeof(Token)+status.N_NODES*status.N_NODES);
	}
	else if	(t->hdr.type==AUTHORIZATION){
		tkn_data_ptr+=(sizeof(Token_Hdr)+sizeof(Authorization));
	}else if	(t->hdr.type==MESSAGE){
		tkn_data_ptr+=(sizeof(Token_Hdr)+sizeof(Message)+t->msg.len);
	}else {
		WMP_MSG(stderr,"Unknown frame type in get_data_pointer, exiting\n");
	}
	return tkn_data_ptr;
}

char * wmp_get_frame_routing_pointer(wmpFrame * t){
	char * tkn_data_ptr =(char*)t;
	if (t->hdr.type==TOKEN){
		tkn_data_ptr+=(sizeof(Token_Hdr)+sizeof(Token));
	}
	else if	(t->hdr.type==AUTHORIZATION){
		tkn_data_ptr+=(sizeof(Token_Hdr)+sizeof(Authorization));
	}else if	(t->hdr.type==MESSAGE){
		tkn_data_ptr+=(sizeof(Token_Hdr)+sizeof(Message)+t->msg.len);
	}else {
		WMP_MSG(stderr,"Unknown frame type in get_data_pointer, exiting\n");
	}
	return tkn_data_ptr;
}

char * wmp_get_message_data_pointer(wmpFrame * t) {
	char * tkn_data_ptr = (char*) t;
	if (t->hdr.type != MESSAGE)
		return NULL;
	tkn_data_ptr += (sizeof(Token_Hdr) + sizeof(Message));
	return tkn_data_ptr;
}

int wmp_get_frame_total_lenght(wmpFrame * t) {
	int size = sizeof(Token_Hdr);
	if (t->hdr.type == TOKEN) {
		size += sizeof(Token) + status.N_NODES * status.N_NODES;
	} else if (t->hdr.type == AUTHORIZATION) {
		size += sizeof(Authorization);
	} else if (t->hdr.type == MESSAGE) {
		if (mBitsIsSet(t->msg.type, AURA_MSG)) {
			size += sizeof(Message) + t->msg.len;
		} else {
			size += sizeof(Authorization);
		}
	} else if (t->hdr.type == DROP_TOKEN) {
		size += sizeof(Drop);
	}
	return size;
}




