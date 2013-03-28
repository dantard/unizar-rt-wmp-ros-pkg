/*------------------------------------------------------------------------
 *---------------------           RT-WMP              --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: ./src/core/routing/basic/manage.c
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

#include "wmp_config.h"
#include "core/include/manage.h"
#include "core/include/flow_control.h"
#include "core/include/queue_core.h"
#include "core/include/aura.h"
#include "core/include/queue_core.h"
#include "core/include/wmp_utils.h"
#include "core/include/task_timing.h"

/*localLQM[i][j]=> node i hear j with quality localLQM[i][j] */
/* node j is heard with quality localLQM[i][j] from node i*/

char isolated_count[32] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

static int confirmed(unsigned int ack, unsigned short ack_hash, short ack_part){
	if (status.wait_ack_of_hash == ack_hash
			&& status.wait_ack_of_part == ack_part
			&& ack == status.wait_ack_from){
		return 1;
	}else{
		return 0;
	}
}

int create_new_token(wmpFrame * t) {
	int i = 0;
	if (status.power_save && t->hdr.waiting <= 1 && status.lastRecvdType != MESSAGE ){
		if (t->hdr.sleep > 0){
			wmp_print("VS ms %d \n",  (int)t->hdr.sleep);
			//return VIGILANT_SLEEP;
			wmpSendAck(t);
			t->hdr.sleep = 0;
			return RECEIVE;
		}
	}
	wmp_print("NT ");
	t->hdr.sleep = ms_to_us(500);
	wmp_print("NPD9:%d ", t->hdr.sleep);

	t->hdr.serial = status.highestSerial;
	t->hdr.to = UNDEF;
	t->hdr.type = FRESH_TOKEN;
	t->hdr.retries = 0;
	t->hdr.from = status.id;
	t->hdr.loop_id += 1;
	t->tkn.maxPri = UNDEF;
	t->tkn.idMaxPri = UNDEF;
	t->tkn.age = 0;
	t->hdr.burst = 0;
	t->hdr.waiting = 0;

	/*clear reached and lost fields*/
	for (i = 0; i < status.N_NODES; i++) {
		nstat_clearReached(i);
	}

	nstat_setReached(status.id);
	nstat_clearLost(status.id);
	status.lr = UNDEF;

	/* Exclude isolated node */
	for (i = 0; i < status.N_NODES; i++) {
		if (isIsolated(lqm_get_ptr(), i)) {
			nstat_setLost(i);
		}
	}

	t->tkn.beginner = status.id;
	return EVALUATE_TOKEN;
}

int evaluate_token(wmpFrame * t) {
	int i, tot, last_one, burst = 0, waiting = 0, delay;
	signed char highestPriority = UNDEF;
	unsigned int age;
	if (t->tkn.idMaxPri >=0){
		delay = task_get_next_predicted_push_delay_most_priority_than(t->tkn.maxPri+1);
	}else{
		delay = task_get_next_predicted_push_delay();
	}

	wmp_print("NPD:%d ", delay);
	if (delay > 1000){ //XXX:number
		delay = 1000;
	}

	delay = ms_to_us(delay);

	if (delay < t->hdr.sleep){
		 t->hdr.sleep = delay;
	}
	wmp_print("NPD2:%d ", delay);
	wmp_print("NPD3:%d ", t->hdr.sleep);
	/* Update the age and the lr (last received) field*/

	aura_clear();

	if ((t->hdr.from != status.id) || (t->hdr.type == FRESH_TOKEN)) {
		wmp_print("ET ");
		t->hdr.type = TOKEN;

        /* Frame freshly received */
		if (t->tkn.idMaxPri != UNDEF) {
			t->tkn.age += wmp_calculate_frame_duration_ms(status.rate,
					wmp_get_frame_total_lenght(t));
		}

		if (!nstat_isReached(status.id)) {
			status.lr = t->hdr.from;
		}
		status.emergency_lr = t->hdr.from;

		/* Reschedule if last message not delivered */
		if (status.enable_message_reschedule) {
			if (status.wait_ack_from) {
				if (confirmed(t->hdr.ack, t->tkn.ack_hash, t->tkn.ack_part)){
					wmp_queue_tx_confirm();
				}else{
					int age, port, period, priority;
					wmp_queue_tx_get_last_popped_info(&age, &port, &priority);
					period = task_get_priority_period(priority);
					if (status.secure || age < period) {
						wmp_print("NPD4:%d ", t->hdr.sleep);

						t->hdr.sleep = 0;
						wmp_queue_tx_reschedule();
					} else{
						wmp_queue_tx_confirm();
					}
				}

				t->hdr.ack = 0;
				t->hdr.fc = 0;
				t->tkn.ack_hash = 0;
				t->tkn.ack_part = 0;
				status.wait_ack_from = 0;
			}
		}else{
			wmp_queue_tx_confirm();
		}

		if (status.loop_id != t->hdr.loop_id){
			status.loop_id = t->hdr.loop_id;
		}

		/* Select MPM */
		waiting = wmp_queue_tx_get_len();
		wmp_print("WAI:%d_HP:%d ",waiting,t->tkn.maxPri);
		if (waiting > 0) {
			int id = wmp_queue_tx_get_head_id();
			highestPriority = wmp_queue_tx_get_elem_priority(id);
			age = wmp_queue_tx_get_elem_age(id);
			burst = wmp_queue_tx_get_elem_burst(id);

			t->hdr.waiting += waiting;

			if (highestPriority > t->tkn.maxPri) {
				/* control if I have a message more priority than the others */
				t->tkn.idMaxPri = status.id;
				t->tkn.maxPri = highestPriority;
				t->tkn.age = age;
				t->hdr.burst = burst;
				wmp_print("MINE_%d ",waiting,t->tkn.maxPri);
			} else if (highestPriority == t->tkn.maxPri) {
				if (t->tkn.age < age) {
					t->tkn.idMaxPri = status.id;
					t->tkn.age = age;
					t->hdr.burst = burst;
				}
			}
		}

		/* I have been reached */
		nstat_setReached(status.id);
		nstat_clearLost(status.id);

		/* I'll send the frame (N.B. Used to see if I'm re-evaluating a frame or not)*/
		t->hdr.from = status.id;

	}

	/*control if I have to search someone --> does not depend on LQM*/
	for (i = 0; i < status.N_NODES; i++) {
		if (i != status.id) {
			if (nstat_isLost(i) && !nstat_isReached(i)) {
				if ((t->hdr.loop_id % wmpGetNumOfNodes()) == status.id) {
					//if (nstat_get_val(i) == status.id) {

					/* I've to search someone!*/
					lqm_set_val(status.id, i, 100);
					lqm_set_val(i, status.id, 100);

					t->hdr.to = i;
					t->hdr.retries = 0;
					t->hdr.type = TOKEN;
					wmp_print("NPD5:%d ", t->hdr.sleep);

					t->hdr.sleep = 0;
					wmp_print("SRCH ");
					return SEND_TOKEN;
				}
			}
		}
	}

	/* Control if I can see someone --> LQM makes decisions*/
	tot = 0;
	for (i = 0; i < status.N_NODES; i++) {
		if (i == status.id) {
			continue;
		}
		tot += lqm_get_val(status.id, i);
	}

	if (tot == 0) {
		wmp_print("NPD6:%d ", t->hdr.sleep);

		t->hdr.sleep = 0;
		status.highestSerial+=status.id*5;
		/* I have the token but cannot see nothing */
		return NEW_TOKEN;
	}

	/*LQM is NOT empty --> Control if I'm the last one*/

	last_one = 1;
	for (i = 0; i < status.N_NODES; i++) {
		last_one = last_one && nstat_isReached(i);
	}
	wmp_print("CLO%d ", t->hdr.sleep);
	if (last_one > 0) {

		/* Force WC */
		//XXX: status.N_NODES > 2
		if (status.id != t->tkn.beginner ) {//&& t->tkn.ack_hash != 0
			nstat_clearReached(t->tkn.beginner);
			wmp_print("CB ");
			return EVALUATE_TOKEN;
		}

		/* I'm the last one */
		if (t->tkn.idMaxPri >= 0) {

			/* if there is a message to transmit */
			return CREATE_AUTHORIZATION;
		} else {
			/* Noone have to transmit nothing - Start a new PAP */
			return NEW_TOKEN;
		}
	} else {
		/* I'm not the last one */
		int bestRssi;
		signed char selected = UNDEF;

		if (status.use_prim) {
			/* new */
			int i, j;
			char ** prim_lqm;

			lqm_backup();

			prim_lqm = prim(lqm_get_ptr());
			for (i = 0; i < status.N_NODES; i++) {
				for (j = 0; j < status.N_NODES; j++) {
					if (i != j) {
						lqm_set_val(i, j, prim_lqm[i][j]);
					}
				}
			}
		} else if (status.use_prune) {
			char ** pruned_lqm;

			lqm_backup();

			pruned_lqm = lqm_prune(lqm_get_ptr());
			lqm_copy_to(lqm_get_ptr(), pruned_lqm);
		}

		/* Chooses next_node to pass the token to */
		bestRssi = 0;
		for (i = 0; i < status.N_NODES; i++) {
			if (i == status.id) {
				continue;
			}
			if (!nstat_isReached(i) && !nstat_isLost(i)) {
				if (lqm_get_val(status.id, i) > bestRssi) {
					bestRssi = lqm_get_val(status.id, i);
					selected = i;
				}
			}
		}

		if (status.use_prim || status.use_prune) {
			lqm_restore();
		}

		if (selected == UNDEF) {
			/* I can't ear anyone that has not been still reached, Go back!
			 * NOTICE: I'm NOT the last one and can't ear noone not reached*/

			if (status.lr == UNDEF) {
				wmp_print("NPD7:%d ", t->hdr.sleep);

				t->hdr.sleep = 0;
				status.highestSerial+=status.id*5;
				return NEW_TOKEN;
			} else {
				/* Going back */
				t->hdr.to = status.lr;
				t->hdr.retries = 0;
				t->hdr.type = TOKEN;
				//TODO: Study this situation
				status.lr = UNDEF;
				wmp_print("ST1 ");
				return SEND_TOKEN;
			}
			/* Execution cannot reach this point */
			ASSERT(0);
		} else {
			/* I can ear someone that has not been still reached, go forward */
			t->hdr.to = selected;
			t->hdr.retries = 0;
			t->hdr.type = TOKEN;
			wmp_print("ST2_%d ", t->hdr.to);
			return SEND_TOKEN;
		}
	}
}

int manage_token_expired_timeout(wmpFrame* t) {/* token timeout expired*/
	/* here we have 2 types of politics:
	 1. retry transmission
	 2. set not responding node to reached ad go on
	 we can retry (1) up to reach 2n-3 hops in total but the risk is that we cannot reach all the nodes.
	 So we can increase this number up to 2n-3+p and provide p>=0 possibility for retry in each loop and only ONE
	 retry per node. It MUST be considered at planification time.
	 */
	if (status.retries < status.maxPerNodeRetries && t->hdr.retries
			< status.maxTotalRetries) {/* set node to reached */
		status.retries++;
		t->hdr.retries++;
		t->hdr.sleep = 0;
		return RETRY;
	} else {
		nstat_setReached(t->hdr.to);
		status.retries = 0;
		//printk(KERN_INFO "TET TO:%d \n", t->hdr.to);

		rssi_reset(t->hdr.to);
		lqm_set_val(status.id, t->hdr.to, 0);
		t->hdr.sleep = 0;
		return EVALUATE_TOKEN;
	}
}

int manage_authorization_expired_timeout(wmpFrame * t) {/* Authorization timeout expired*/
	if (status.retries < status.maxPerNodeRetries && t->hdr.retries
			< status.maxTotalRetries) {/* set node to reached */
		status.retries++;
		t->hdr.retries++;
		t->hdr.sleep = 0;
		wmp_print("RTR ");
		return RETRY;
	} else {
		status.retries = 0;
		//printk(KERN_INFO "MAT TO:%d \n", t->hdr.to);

		rssi_reset(t->hdr.to);
		lqm_set_val(status.id, t->hdr.to, 0);
		t->hdr.sleep = 0;
		wmp_print("RTRNT ");
		return NEW_TOKEN;
	}
}

int manage_message_expired_timeout(wmpFrame * t) {/* Authorization timeout expired*/
	if (status.retries < status.maxPerNodeRetries && t->hdr.retries
			< status.maxTotalRetries) {/* set node to reached */
		status.retries++;
		t->hdr.retries++;
		t->hdr.sleep = 0;
		return RETRY;
	} else {
		status.retries = 0;
		//printk(KERN_INFO "MET TO:%d \n", t->hdr.to);

		rssi_reset(t->hdr.to);
		lqm_set_val(status.id, t->hdr.to, 0);
		t->hdr.sleep = 0;
		return NEW_TOKEN;
	}
}

int evaluate_foreign(wmpFrame * t) {
	return RECEIVE;
}

static signed char getNext(wmpFrame * t) {

	signed char next = UNDEF;
	unsigned int reached = 0;
	int i, j, dest = UNDEF, cost;

	if (t->hdr.type == MESSAGE) {
		dest = t->msg.dest;
		reached = t->msg.reached;
	} else if (t->hdr.type == AUTHORIZATION) {
		dest = t->aut.dest;
		reached = t->aut.reached;
	}
	if (status.use_prim) {
		next = nextStepWithCost(prim(lqm_get_ptr()), status.id, dest, &cost);
	} else if (status.use_prune){
		next = nextStepWithCost(lqm_prune(lqm_get_ptr()), status.id, dest, &cost);
	}

	if (next == UNDEF || mBitsIsSet(reached,next)) {
		lqm_backup();
		for (i = 0; i < status.N_NODES; i++) {
			if (i == status.id) {
				continue;
			}
			if (mBitsIsSet(reached,i)) {
				for (j = 0; j < status.N_NODES; j++) {
					lqm_set_val(i, j, 0);
					lqm_set_val(j, i, 0);
				}
			}
		}
		next = nextStepWithCost(lqm_get_ptr(), status.id, dest, &cost);
		lqm_restore();
	}
	return next;
}

int evaluate_message(wmpFrame * t) {
	int i, more = 0;

	if (mBitsIsSet(t->msg.type, BURST)){
		aura_clear();
	}
	wmp_print("MDST:%u ",t->msg.dest);

	if (t->hdr.from != status.id) {
		if (mBitsIsSet(t->msg.type, AURA_MSG)){
			aura_add(aura_msg, t->hdr.from);
		}else{
			aura_add(aura_auth, t->hdr.from);
		}
	}

	if (mBitsIsSet(t->msg.type,AURA_MSG)){
		aura_store_msg(&t->msg);
	}

	aura_discard_unnecessary(t->msg.dest);

	mBitsSet(t->msg.reached,status.id);
	/* new */

	if (mBitsIsSet(t->msg.type,AURA_MSG)){
		if (mBitsIsSet(t->msg.dest,status.id)) {//is for me
			mBitsUnset(t->msg.dest,status.id);

			enqueue_message(t);
			mBitsSet(t->hdr.ack, wmpGetNodeId());

			/* Flow control */
			if (wmp_queue_rx_get_room() > 5) {
				mBitsSet(t->hdr.fc, wmpGetNodeId());
			}else{
				mBitsUnset(t->hdr.fc, wmpGetNodeId());
			}
		}
	}

	for (i = 0; i<status.N_NODES ;i++){
		more = more || mBitsIsSet(t->msg.dest,i);
	}

	wmp_print("MORE:%d ",more);
	if (more){
		aura_t type;
		int next = aura_get_next(t,&type);

		if (type == aura_auth){

			mBitsUnset(t->msg.type,AURA_MSG);
			wmp_print("AUAUTH ");

		} else{
			mBitsSet(t->msg.type,AURA_MSG);
			aura_restore_msg(&t->msg);
			wmp_print("AUMSG ");

			if (next < 0 || mBitsIsSet(t->msg.reached,next)) {
				t->msg.reached = 0;
				wmp_print("NXT:%d MBIS:%d ", next, mBitsIsSet(t->msg.reached,next));
				return NEW_TOKEN;
			}
		}

		if (next >= 0) {
			t->hdr.from = status.id;
			t->hdr.to = next;
			t->hdr.retries = 0;
			t->hdr.type = MESSAGE;

			aura_add(type, t->hdr.to);

			t->msg.age += wmp_calculate_frame_duration_ms(status.rate,
					wmp_get_frame_total_lenght(t));
			wmp_print("EMSM ");
			return SEND_MESSAGE;
		}
	}
	wmp_print("SLP:%d_MS:%d_R:%d ", t->hdr.sleep, ms_to_us(5),t->hdr.sleep > ms_to_us(5));
	if (status.power_save && t->hdr.burst > 1 && t->hdr.sleep > ms_to_us(5)){ //XXX:number
		signed char msg_src = t->msg.src;
		unsigned short ack_hash = t->msg.msg_hash;
		signed char ack_part_id = t->msg.part_id;
		t->hdr.from = status.id;
		t->hdr.type = AUTHORIZATION;
		t->hdr.retries = 0;
		t->hdr.burst = t->hdr.burst - 1;
		mBitsSet(t->aut.type, BURST);
		t->aut.dest = msg_src;
		t->aut.ack_hash = ack_hash;
		t->aut.ack_part = ack_part_id;
		wmp_print("EMEA ");
		return EVALUATE_AUTHORIZATION;
	}else{
		unsigned short ack_hash = t->msg.msg_hash;
		signed char ack_part_id = t->msg.part_id;
		t->msg.reached = 0;
		t->tkn.ack_hash = ack_hash;
		t->tkn.ack_part = ack_part_id;
		wmp_print("EMNT ");
		return NEW_TOKEN;
	}
}

int evaluate_authorization(wmpFrame * t) {
	if (t->aut.dest == UNDEF){
		wmp_print("EANT ");
		return NEW_TOKEN;
	}

	if (mBitsIsSet(t->aut.type, BURST)){
		aura_clear();
	}

	if (t->hdr.from == status.id) {
		t->aut.reached = 0;
	} else {
		aura_add(aura_auth, t->hdr.from);
	}
	mBitsSet(t->aut.reached,status.id);

	if (t->aut.dest == status.id) {/* Is for me */

		/* Reschedule */
		if (status.enable_message_reschedule) {
			if (status.wait_ack_from) {
				if (confirmed(t->hdr.ack, t->aut.ack_hash, t->aut.ack_part)){
					wmp_queue_tx_confirm();
				}else{
					int age, port, period, priority;
					wmp_queue_tx_get_last_popped_info(&age, &port, &priority);
					period = task_get_priority_period(priority);
					if (status.secure || age < period) {
						wmp_print("NPD8:%d ", t->hdr.sleep);

						t->hdr.sleep = 0;
						wmp_queue_tx_reschedule();
					} else{
						wmp_queue_tx_confirm();
					}
				}
				t->hdr.ack = 0;
				t->hdr.fc = 0;
				t->tkn.ack_hash = 0;
				t->tkn.ack_part = 0;
				status.wait_ack_from = 0;
			}
		}else{
			wmp_queue_tx_confirm();
		}
		if (wmp_queue_tx_get_count() > 0) {
			wmp_print("EACM ");
			return CREATE_MESSAGE;
		} else {
			wmp_print("EANT2 ");
			return NEW_TOKEN; /* NO waiting messages */
		}
	} else { /* The message is not for me */
		int next = getNext(t);
		if (next >= 0) {
			if (mBitsIsSet(t->aut.reached,next)) {
				return NEW_TOKEN;
			} else {
				t->hdr.from = status.id;
				t->hdr.to = next;
				t->hdr.retries = 0;
				t->hdr.type = AUTHORIZATION;
				aura_add(aura_auth, t->hdr.to);
				wmp_print("EASA ");
				return SEND_AUTHORIZATION;
			}
		} else {
			wmp_print("EANT3 ");
			return NEW_TOKEN; /* No path to destination*/
		}
	}
}

int create_authorization(wmpFrame * t) {
	t->aut.dest = t->tkn.idMaxPri;
	t->aut.src = status.id;
	t->hdr.from = status.id;
	t->hdr.type = AUTHORIZATION;
	t->hdr.retries = 0;
	mBitsUnset(t->aut.type,BURST);
	return EVALUATE_AUTHORIZATION;
}

int create_message(wmpFrame * t) {
   static longMsg_t * m;
	int m_id = wmp_queue_tx_pop_part(&m);

	t->hdr.burst = wmp_queue_tx_get_elem_burst(m_id);

	t->hdr.type = MESSAGE;
	t->hdr.from = status.id;
	t->hdr.to = status.id; /* like If I received the frame */

	t->msg.age =  us_to_ms(((int)(getRawActualTimeus()-m->ts)));
	t->msg.dest = m->dest;

	t->msg.src = status.id;
	memcpy(wmp_get_message_data_pointer(t), m->pointer, m->this_part_size);
	t->msg.len = (unsigned short) m->this_part_size;
	t->msg.port = m->port;
	t->msg.priority = m->priority;

	t->msg.msg_hash = m->hash;
	t->msg.part_id = m->part_id;
	t->msg.msg_part_size = m->msg_part_size;

	t->msg.reached = 0;

	mBitsSet(t->msg.type,AURA_MSG);
	if (m->rescheduled){
		mBitsSet(t->msg.type,RESCHEDULED);
	}else{
		mBitsUnset(t->msg.type,RESCHEDULED);
	}

	t->hdr.ack = 0;
	t->hdr.fc = 0;
	status.wait_ack_from = m->dest;
	status.wait_ack_of_hash = m->hash;
	status.wait_ack_of_part = m->part_id;

	wmp_queue_tx_pop_part_done(m_id);
	return EVALUATE_MESSAGE;
}

int enqueue_message(wmpFrame * t) {
    static longMsg_t m;
    m.src = t->msg.src;
	m.priority = t->msg.priority;
	m.dest = t->msg.dest;
	m.size = t->msg.len;
	m.data = wmp_get_message_data_pointer(t);
	m.port = t->msg.port;
	m.hash = t->msg.msg_hash;
	m.part_id = t->msg.part_id;
	m.msg_part_size = t->msg.msg_part_size;
	wmp_queue_rx_push_part(&m);
	return NEW_TOKEN;
}

