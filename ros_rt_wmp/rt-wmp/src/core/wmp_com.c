/*------------------------------------------------------------------------
 *---------------------           RT-WMP              --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: ./src/core/wmp_com.c
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

#include "config/compiler.h"
#include "core/include/wmp_com.h"
#include "core/include/wmp_misc.h"
#include "core/include/wmp_utils.h"
#include "core/include/global.h"
#include "core/include/wmp_com.h"
#include "core/include/ml_com.h"

static char (*fp) (char);


struct {
	unsigned char to;
	unsigned char from;
	unsigned char type;
	int serial;
} lastRX, lastTX ;


char f_rssi(char rssi){
      long long ll_rssi = rssi;
      if (ll_rssi > 100) {
         ll_rssi = 100;
      }
      return (char) ll_rssi;
}

char (*rssi_get_f())(char){
	return fp;
}

void rssi_set_f( char (*f) (char)){
	if (f != 0){
		fp = f;
	} else{
		f = f_rssi;
	}
}

char getSimulatedRssi(char to){
	return 80;
}

void wmpUpdateRssi(wmpFrame *p){
	int i;
	/* Calculate RSSI */

	p->hdr.rssi= fp(p->hdr.rssi);
	rssi_new_frame(p->hdr.from, p->hdr.rssi);

	//status.retries=0;
	for (i=0;i<status.N_NODES;i++){
		if (i==status.id) {
			continue;
		}
		if (rssi_get_averaged_rssi(i)>0 && rssi_get_age(i)> status.hold_time){
			rssi_reset(i);
		}
	}
	/* if I received a frame but the other says does not hear me, I suppose it
	 * hear me
	 */
	rssi_new_loop(p->hdr.from, p->hdr.loop_id);

	lqm_set_val(status.id,p->hdr.from,rssi_get_averaged_rssi(p->hdr.from));
}

void wmpSendDrop(wmpFrame * p){
	wmpFrame q;
	q.hdr.to=p->hdr.from;
	q.hdr.from=status.id;
	q.hdr.type=DROP_TOKEN;
	q.hdr.rssi=getSimulatedRssi(q.hdr.to);
	q.hdr.noise=0;
	q.hdr.retries=0;
	q.drop.drop_serial=p->hdr.serial;
	q.hdr.serial=status.highestSerial;
	status.highestSerial+=2;
	ml_send(&q,sizeof(Token_Hdr)+sizeof(Drop));
}

int wmpReceive(wmpFrame* q){
   int rtn;
	status.rx_type=0;
	rtn=ml_receive(q, status.net_inactivity_timeout);
	if (rtn==EXPIRED){
		WMP_DBG(WMP_COM,"*** WARNING :: RECEIVE EXPIRED (%d ms)\n",status.net_inactivity_timeout);
		status.highestSerial+=100;
		status.reason |= 64;
		return NET_INACTIVITY;
	}else {
		return UPDATE_RSSI_ON_RECEIVE;
	}
}

int wmpUpdateReceivedRssi(wmpFrame* q){
	wmpUpdateRssi(q);
	return DECODE_ROUTING_INFO_ON_RECEIVE;
}

int wmpUpdateAcknowkedgedRssi(wmpFrame* q){

	rssi_confiability_increment(q->hdr.from);
	wmpUpdateRssi(q);
	return DECODE_ROUTING_INFO_ON_WACK;
}

int wmpWaitAck(wmpFrame* q){
	int rtn=ml_receive(q, status.waitRemaining); //<- filled in send
	if (rtn==RECEIVE_OK){ /* something received*/
		//status.retries=0; //SEPT08
		return UPDATE_RSSI_ON_ACK; /* -> INTERPRET_ACK */
	}else{
		return (TO_EXPIRED + status.lastSentType);
	}
}


int wmpSend(wmpFrame* p){

	int duration, size = wmp_get_frame_total_lenght(p);

	status.highestSerial++;
	status.lastSentType=p->hdr.type;
	status.waitRemaining=status.TIMEOUT;
	status.wait_implicit_ack_from = p->hdr.to;

	/* MANAGE POWER SAVE */
	duration = wmp_calculate_frame_duration_us(status.rate,size);

	if (p->hdr.sleep > duration){
		p->hdr.sleep -= duration;
	}else{
		p->hdr.sleep = 0;
	}

	/* < MANAGE POWER SAVE */

	p->hdr.rssi = getSimulatedRssi(p->hdr.to);
	p->hdr.from = status.id;
	p->hdr.serial = status.highestSerial;

	usleep(status.cpu_delay);

	if (p->hdr.type > 4) {
		WMP_MSG(stderr,"*** WARNING: UNKNOWN TYPE on SEND\n");
	}

	lastTX.from = p->hdr.from;
	lastTX.to = p->hdr.to;
	lastTX.serial = p->hdr.serial;
	lastTX.type = p->hdr.type;

	size+=wmp_print_put(p);
	ml_send( p, size);
	return WAIT_ACK;
}

void wmp_send_setup(wmpFrame * p){

}

void initComLayer(){
	fp = f_rssi;
}

int wmp_send_retry(wmpFrame * p){
	status.waitRemaining = status.TIMEOUT;
	status.highestSerial--;
	return wmpSend(p);
}

int wmpInterpretAck(wmpFrame **p, wmpFrame **q){

	int isForMe = (*q)->hdr.to == status.id;
	int isValid = (*q)->hdr.serial > status.highestSerial;
	int isDrop  = (*q)->hdr.type == DROP_TOKEN;
	int isAck  = (*q)->hdr.type == ACK;

	if (isForMe && isAck) {
		return VIGILANT_SLEEP;
	}

	if (!isValid) {
		return WAIT_ACK;
	}

	if (isForMe && isDrop) {
		status.highestSerial = (*q)->hdr.serial;
		return RECEIVE;
	}

	if (isForMe && !isDrop) {
		wmpFrame * t;
		t = *q; *q = *p; *p = t;
		status.highestSerial=(*p)->hdr.serial;
		status.lastRecvdType = (*p)->hdr.type;
		status.lastRecvdFrom = (*p)->hdr.from;
		return (EVALUATE + (*p)->hdr.type);
	}

	if (!isForMe && !isDrop) {
		status.highestSerial=(*q)->hdr.serial;
		return EVALUATE_FOREIGN;
	}

	if (!isForMe && isDrop) {
		return WAIT_ACK;
	}

	ASSERT(CANT_ARRIVE);
	return CANT_ARRIVE;
}

int wmpInterpretReceived(wmpFrame **p, wmpFrame **q){

	int isForMe = (*q)->hdr.to == status.id;
	int isValid = (*q)->hdr.serial > status.highestSerial;
	int isDrop  = (*q)->hdr.type == DROP_TOKEN;

	int isAck  = (*q)->hdr.type == ACK;
	if (isForMe && isAck) {
		return VIGILANT_SLEEP;
	}

	if (!isValid) {
		return RECEIVE;
	}

	if (isForMe && isDrop) {
		status.highestSerial = (*q)->hdr.serial;
		return RECEIVE;
	}

	if (isForMe && !isDrop) {
		wmpFrame * t;
		t = *q; *q = *p; *p = t;
		status.highestSerial=(*p)->hdr.serial;
		status.lastRecvdType = (*p)->hdr.type;
		status.lastRecvdFrom = (*p)->hdr.from;
		return (EVALUATE + (*p)->hdr.type);
	}

	if (!isForMe && !isDrop) {
		status.highestSerial=(*q)->hdr.serial;
		return EVALUATE_FOREIGN;
	}

	if (!isForMe && isDrop) {
		return RECEIVE;
	}

	ASSERT(CANT_ARRIVE);
	return CANT_ARRIVE;
}

void wmpSendAck(wmpFrame * p){
	int size;
	unsigned short ack_hash;
	short ack_part;
	if (p->hdr.type == MESSAGE){
		ack_hash = p->msg.msg_hash;
		ack_part = p->msg.part_id;
	} else if(p->hdr.type == TOKEN){
		ack_hash = p->tkn.ack_hash;
		ack_part = p->tkn.ack_part;
	}else if(p->hdr.type == AUTHORIZATION){
		ack_hash = p->aut.ack_hash;
		ack_part = p->aut.ack_part;
	}else{
		ack_hash = 0;
		ack_part = 0;
	}
	p->ack.ack_hash = ack_hash;
	p->ack.ack_part = ack_part;

	p->hdr.to=status.lastRecvdFrom;
	p->hdr.from=status.id;
	p->hdr.type=ACK;
	p->hdr.rssi=getSimulatedRssi(p->hdr.to);
	p->hdr.noise=0;
	p->hdr.retries=0;
	status.highestSerial++;
	p->hdr.serial=status.highestSerial;
	size = sizeof(Token_Hdr)+sizeof(Ack)+wmp_print_put(p);
	ml_send( p, size);
}

int vigilant_sleep(wmpFrame * p, wmpFrame * q){
	int rtn;
	status.highestSerial = q->hdr.serial;
	rtn=ml_receive(q, q->hdr.sleep/1000); ///XXXX should be q
	if (rtn==EXPIRED){
		p->hdr.sleep = 0;
		return NEW_TOKEN;
	}else {
		q->hdr.sleep = 0;
		p->hdr.sleep = 0;
		return INTERPRET_RECEIVED;
	}
}
