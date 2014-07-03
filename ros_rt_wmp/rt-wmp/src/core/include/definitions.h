/*------------------------------------------------------------------------
 *---------------------           RT-WMP              --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: ./src/core/include/definitions.h
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



#ifndef DEFINITIONS__H
#define DEFINITIONS__H

#include "core/interface/Msg.h"

#define CANT_ARRIVE 0
#define FORCE_WC_LOOP 34

/*FUNCTION RETURN CODE*/
#define RECEIVE_OK  44
#define ERROR		-99		/*generic error*/
#define EXPIRED		-2
#define UNDEF		((signed char) -1)

/*STATES*/
#define EVALUATE 					10
#define EVALUATE_TOKEN				12
#define EVALUATE_AUTHORIZATION		13
#define EVALUATE_MESSAGE			14
#define EVALUATE_DROP_TOKEN			15
#define EVALUATE_FOREIGN			16

#define TO_EXPIRED					20
#define TO_DISCOVER_EXPIRED			21
#define TO_TOKEN_EXPIRED			22
#define TO_AUTHORIZATION_EXPIRED	23
#define TO_MESSAGE_EXPIRED			24

#define NEW_TOKEN					32
#define WAIT_ACK					33
#define RETRY						35
#define RECEIVE						36
#define DUPLICATE_TOKEN				37
#define INTERPRET_RECEIVED			39

#define INTERPRET_ACK					40
#define ENQUEUE_MESSAGE					41
#define CREATE_AUTHORIZATION			45
#define CREATE_MESSAGE  				46
#define SEND_DROP_ON_ACK				47
#define SEND_DROP_ON_RECEIVE			48
#define DECODE_ROUTING_INFO_ON_RECEIVE	49
#define DECODE_ROUTING_INFO_ON_WACK		50

#define SEND_TOKEN						51
#define SEND_MESSAGE					52
#define SEND_AUTHORIZATION				53

#define UPDATE_RSSI_ON_RECEIVE			54
#define UPDATE_RSSI_ON_ACK				55
#define VIGILANT_SLEEP					56

#define NET_INACTIVITY 					65
#define ABORT							66
#define ALL_STATES						127

/* RT-WMP Extensions */
#define NO_EXTESION					0
#define NERUS						1
#define PLUS						2



/* PROGRAM STATUS*/
typedef struct  {
/* Node ID and working Parameters*/
	int id;
	char N_NODES;
	long timeout;
	int hold_time;
	int rx_queue_elements;
	int tx_queue_elements;
/* Working Buffers*/
	char bufferA[MTU];
	char bufferB[MTU];
/*Local Information*/
	unsigned long highestSerial;
	int lastSentType;
	int lastRecvdType;
	int lastRecvdFrom;
	signed char lastAckd;
	int retries;

	int maxPerNodeRetries;
	int maxTotalRetries;
	int waitRemaining;
	signed char acknow;
	char reincorporation_request;
/*Configuration Parameters*/
	int TIMEOUT;
	int perHopDelay;
	int max_rssi;
	signed char lr;
	signed char emergency_lr;
	int cpu_delay;
	int quiet;
	int mobile_average_elements;
	unsigned int net_inactivity_timeout;
	int rx_type;
	int rssi_rising_factor;
	int rate;
	int reason;
	char status[32];
	int multiplier;	/* Worst Case Multiplier */
	int wait_ack_from;

	unsigned short wait_ack_of_hash;
	short wait_ack_of_part;
	signed char wait_implicit_ack_from;
	char send_ack_to;
	int enable_flow_control;
	int enable_message_reschedule;
	unsigned int loop_id;
	unsigned int serial;
	unsigned char net_id;
	int use_ett;
	int use_prim;
	int use_prob;
	int use_prune;
	int MAXIMUM_DATA_SIZE;
	double x,y;
	int pow;
	int active_search;
	short instance_id;
	int w100;
	int w3;
	int w2;
	int w1;
	int power_save;
	int burst;
	int secure;
	int max_msg_size;
	int num_ports;
	int beluga;
	int prune_threshold;
	int prob_99_perc_rssi_min;
	int use_aura_efficient_multicast;
}Status;


//#define SV int states_vector=[][]{


#endif

