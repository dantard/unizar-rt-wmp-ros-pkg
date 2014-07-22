/*------------------------------------------------------------------------
 *---------------------           RT-WMP              --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: ./src/core/include/frames.h
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

#include <linux/types.h>
#include <asm/types.h>

#ifndef FRAMES_H_
#define FRAMES_H_
/* STATES TYPES */
#define FRESH_TOKEN     1
#define TOKEN           2
#define AUTHORIZATION 	3
#define MESSAGE         4
#define DROP_TOKEN      5
#define ACK			    6

#define BURST			1
#define RESCHEDULED		2
#define AURA_MSG		4

typedef struct {
	__s8 rssi;
	__s8 noise;
	__u32 serial;
	__u8 net_id;
	__s8 type;
	__s8 from;
	__s8 to;
	__s8 retries;
	__u32 ack;
	__u16 loop_id;
	__u16 burst;
	__s32 sleep;
	__u8 waiting;
	__u32 fc;
	__u32 gretries;
} __attribute__ ((__packed__)) Token_Hdr;

typedef struct {
	__u8 beginner;
	__s8 maxPri;
	__s8 idMaxPri;
	__u32 age;
	__u16 ack_hash;
	__s16 ack_part;
} __attribute__ ((__packed__)) Token;

typedef struct {
	__u8 type;
	__u8 src;
	__u32 dest;
	__u32 reached;
	__u32 age;
	__u16 msg_hash;
	__u16 msg_part_size;
	__s16 part_id;
	__u16 len;
	__u8 port;
	__s8 priority;
	__s8 path[32];
	__s8 rssi[32];
	__u32 cnt;
} __attribute__ ((__packed__)) Message;

typedef struct {
	__u8 type;
	__u8 src;
	__u32 dest;
	__u32 reached;
	__u32 age;
	__u16 ack_hash;
	__s16 ack_part;
} __attribute__ ((__packed__)) Authorization;

typedef struct {
	__u32 drop_serial;
} __attribute__ ((__packed__)) Drop;

typedef struct {
	__u16 ack_hash;
	__s16 ack_part;
} __attribute__ ((__packed__)) Ack;

typedef struct {
	Token_Hdr hdr;
	union {
		Token tkn;
		Message msg;
		Authorization aut;
		Drop drop;
		Ack ack;
	};
} __attribute__ ((__packed__)) wmpFrame;

char * wmp_get_frame_tail_pointer(wmpFrame * t);
char * wmp_get_frame_routing_pointer(wmpFrame * t);
char * wmp_get_message_data_pointer(wmpFrame * t);
int wmp_get_frame_total_lenght(wmpFrame * t);
char * wmp_get_frame_routing_pointer_new_tree(wmpFrame * t);
char * wmp_get_frame_routing_pointer_status(wmpFrame * t);
#endif /*FRAMES_H_*/

