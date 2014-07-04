/*------------------------------------------------------------------------
 *---------------------           RT-WMP              --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: ./src/core/RT-WMP.c
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
#include "core/include/queues.h"
#include "include/rssi_average.h"
#include "include/wmp_com.h"
#include "include/manage.h"
#include "include/wmp_misc.h"
#include "interface/wmp_interface.h"
#include "include/nstat.h"
#include "include/lqm.h"
#include "include/prim.h"
#include "include/argon.h"
#include "include/ll_com.h"
#include "include/wmp_com.h"
#include "include/flow_control.h"
#include "include/kernel.h"
#include "include/queue_core.h"

int wmpReadConfiguration(Status *);
int wmpInit(void);

#define RS_DOWN 			0
#define RS_INITIALIZED 		1
#define RS_RUNNING			2

#define FALSE 0
#define TRUE 1

Status status;
static int abort_requested = 0;
static int running_state = RS_DOWN;

static SEM_T sem;
static SEM_T barrier;

static THREAD_T(th);
static wmpFrame *p, *q; /* buffer and rxbuffer */


void close_all(void) {

	CLOSE_PROC();

	closeLowLevelCom();
	free_lqm();
	freeFlowControl();
	free_nstat();
	wmp_queue_free();
	free_prim();
	freeDijkstra();
	freeMobileAverage();
	WMP_MSG(stderr,"\n*** RT-WMP NODE %d TERMINATED.\n",status.id);
	running_state = RS_DOWN;
}

int wmpSetupList(int _node_id, int _num_nodes, int nparam, ...) {
	int i;
	va_list pa;
	wmpSetDefaultConfiguration(&status);
	wmpReadConfiguration(&status);

	va_start(pa, nparam);
	if (_node_id >= 0) {
		status.id = _node_id;
	}
	if (_num_nodes > 0) {
		status.N_NODES = _num_nodes;
	}
	for (i = 0; i < nparam; i += 2) {
		char * p = va_arg(pa,char *);
		int v = va_arg(pa,int);
		if (strcmp(p, "rx-queue-size") == 0) {
			WMP_MSG(stderr,"Setting RX queue-size = %d\n",v);
			status.rx_queue_elements = v;
		}
		if (strcmp(p, "tx-queue-size") == 0) {
			WMP_MSG(stderr,"Setting RX queue-size = %d\n",v);
			status.tx_queue_elements = v;
		}
	}
	return wmpInit();
}

int wmpSetupArg(char argc, char *argv[]) {

	WMP_DBG(CORE,"RT-WMP Node Starting... \n");

	wmpSetDefaultConfiguration(&status);
	wmpReadConfiguration(&status);

	argo_setCommentId(argo_addInt(&status.id, "node-id", 0, 1),
			"Specify node WMP address");
	argo_setCommentId(argo_addInt((int*) &status.N_NODES, "num-nodes", 2, 1),
			"Specify the number of network node");
	argo_setCommentId(argo_addInt(&status.TIMEOUT, "timeout", 2, 1),
			"Specify the ACK timeout");

	argo_doProcess(argc, argv, 0);

	return wmpInit();

}

int wmpSetup(char _node_id, char _num_nodes) {

	WMP_DBG(CORE,"RT-WMP Node Starting... \n");

	wmpSetDefaultConfiguration(&status);
	wmpReadConfiguration(&status);

	if (_node_id >= 0 && _node_id < _num_nodes) {
		status.id = _node_id;
	}
	if (_num_nodes > 0) {
		status.N_NODES = _num_nodes;
	}

	return wmpInit();
}

int wmpInit(void) {
	p = (wmpFrame *) &(status.bufferA[0]);
	q = (wmpFrame *) &(status.bufferB[0]);

	memset(status.bufferA, 0, MTU);
	memset(status.bufferB, 0, MTU);

	init_lqm(status.N_NODES);
	fill_lqm(0);

	initDijkstra(status.N_NODES);
	init_prim(status.N_NODES);
	wmp_queue_init(status.rx_queue_elements, status.tx_queue_elements, status.max_msg_size, status.num_ports);
	initMobileAverage(status.mobile_average_elements);
	init_nstat(status.N_NODES);
	initFlowControl(status.N_NODES);
	initComLayer();

	if (!initLowLevelCom()) {
		WMP_MSG(stderr,"Low Level Communication Layer Initialization error\n");
		free_lqm();
		freeFlowControl();
		free_nstat();
		wmp_queue_free();
		free_prim();
		freeDijkstra();
		freeMobileAverage();
		running_state = RS_DOWN;
		return FALSE;
	}

	if (!INIT_PROC()) {
		WMP_MSG(stderr,"Proc files initialization error, exiting.\n");
		closeLowLevelCom();
		free_lqm();
		freeFlowControl();
		free_nstat();
		wmp_queue_free();
		free_prim();
		freeDijkstra();
		freeMobileAverage();
		running_state = RS_DOWN;

		return FALSE;
	}


	if (status.net_id == 0) {
		status.net_id = status.N_NODES;
	}
	status.net_inactivity_timeout = 1000 + status.id * 5000;

	abort_requested = 0;

	SEM_INIT(&sem,0,1); /* Init syncro semaphore */
	SEM_INIT(&barrier,0,0); /* Init syncro semaphore */

	status.MAXIMUM_DATA_SIZE = MTU - sizeof(Token_Hdr) - sizeof(Message) - 1;

	/* Init control fields */
	status.lr = UNDEF;
	running_state = RS_INITIALIZED;

	WMP_MSG(stderr,"\n*** Node %d of %d is up and Running...\n",status.id,status.N_NODES);

	return TRUE;
}

//FILE * fp = 0;

#ifndef __KERNEL__
	void* main_loop(void* param) {
#else
	int main_loop(void* param) {
#endif

	int rtnCode = RECEIVE;
	int state = rtnCode;
	//WAIT(sem);
	SIGNAL(barrier);
	running_state = RS_RUNNING;

#ifndef __KERNEL__
	while (rtnCode != ABORT) {
		if (abort_requested) {
			rtnCode = ABORT;
		}
#else
		while(!kthread_should_stop()) {
#endif
		state = rtnCode;
		status.serial = p->hdr.serial;
//		if (fp==0){
//			fp = fopen("rt-wmp-states.dat","a+");
//		}
//		fprintf(fp,"%d %d %d",rtnCode, p->hdr.from, p->hdr.to);
//		fflush(fp);
//		fprintf(stderr,"STATE: %d\n",rtnCode);
		switch (rtnCode) {
		case RECEIVE:
			rtnCode = wmpReceive(q);
			break;
		case INTERPRET_RECEIVED:
			rtnCode = wmpInterpretReceived(&p, &q);
			break;
		case WAIT_ACK:
			rtnCode = wmpWaitAck(q);
			break;
		case INTERPRET_ACK:
			rtnCode = wmpInterpretAck(&p, &q);
			break;
		case EVALUATE_TOKEN:
			rtnCode = evaluate_token(p);
			break;
		case EVALUATE_AUTHORIZATION:
			rtnCode = evaluate_authorization(p);
			break;
		case EVALUATE_MESSAGE:
			rtnCode = evaluate_message(p);
			break;
		case EVALUATE_DROP_TOKEN:
			rtnCode = RECEIVE;
			break;
		case EVALUATE_FOREIGN:
			rtnCode = evaluate_foreign(q);
			break;
		case NEW_TOKEN:
			rtnCode = create_new_token(p);
			break;
		case CREATE_AUTHORIZATION:
			rtnCode = create_authorization(p);
			break;
		case CREATE_MESSAGE:
			rtnCode = create_message(p);
			break;
		case TO_TOKEN_EXPIRED:
			rtnCode = manage_token_expired_timeout(p);
			break;
		case TO_AUTHORIZATION_EXPIRED:
			rtnCode = manage_authorization_expired_timeout(p);
			break;
		case TO_MESSAGE_EXPIRED:
			rtnCode = manage_message_expired_timeout(p);
			break;
		case SEND_TOKEN:
			encode_routing_info(p);
			rtnCode = wmpSend(p);
			break;
		case SEND_AUTHORIZATION:
			//encode_routing_info(p);
			rtnCode = wmpSend(p);
			break;
		case SEND_MESSAGE:
			//encode_routing_info(p);
			rtnCode = wmpSend(p);
			break;
		case ENQUEUE_MESSAGE:
			rtnCode = enqueue_message(p);
			break;
		case RETRY:
			rtnCode = wmp_send_retry(p);
			break;
		case SEND_DROP_ON_ACK:
			wmpSendDrop(q);
			rtnCode = WAIT_ACK;
			break;
		case SEND_DROP_ON_RECEIVE:
			wmpSendDrop(q);
			rtnCode = RECEIVE;
			break;
		case DECODE_ROUTING_INFO_ON_RECEIVE:
			decode_routing_info(q);
			rtnCode = INTERPRET_RECEIVED;
			break;
		case DECODE_ROUTING_INFO_ON_WACK:
			decode_routing_info(q);
			rtnCode = INTERPRET_ACK;
			break;
		case UPDATE_RSSI_ON_RECEIVE:
			rtnCode = wmpUpdateReceivedRssi(q);
			break;
		case UPDATE_RSSI_ON_ACK:
			rtnCode = wmpUpdateAcknowkedgedRssi(q);
			break;
		case NET_INACTIVITY:
			rtnCode = NEW_TOKEN;
			WMP_DBG(CORE,"new_inactivity node %d\n", wmpGetNodeId());
			break;
		case ABORT:
			break;
		case VIGILANT_SLEEP:
			rtnCode = vigilant_sleep(p,q);
			break;
		default:
			WMP_MSG(stderr,"Unmanaged State (%d) at main loop. Application will exit\n",rtnCode);
			EXIT(0);
		}
	}

	th = 0;
	SIGNAL(sem);
#ifndef __KERNEL__
	return (void*) NULL;
#else
	return 0;
#endif
}

void wmpRunBG(void) {
	if (th == 0) {
		THREAD_CREATE(th,main_loop,"main_loop");
		WAIT(barrier);
	}
}

void wmpRun(void) {
	main_loop(NULL);
}

void wmpExit(void) {

	THREAD_STOP(th); // Stops the thread and frees its structure

	abort_requested = TRUE;
	if (running_state == RS_INITIALIZED || running_state == RS_RUNNING) {
		WAIT(sem);
		close_all();
	}
}

void wmpInmediateExit(void) {
	if (running_state != RS_DOWN)
		close_all();
}

void wmpSetQuiet(void) {
	status.quiet = TRUE;
}


