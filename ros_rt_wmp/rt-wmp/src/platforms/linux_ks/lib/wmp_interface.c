/*-------------------------------------------------------------------------
 *--------------------------- RT-WMP IP INTERFACE -------------------------
 *-------------------------------------------------------------------------
 *
 * File: wmp_interface.c
 * Authors: Rubén Durán
 *          Danilo Tardioli
 *-------------------------------------------------------------------------
 *  Copyright (C) 2011, Universidad de Zaragoza, SPAIN
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *-----------------------------------------------------------------------*/

#include "core/interface/wmp_interface.h"
#include <stdio.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <linux/if.h>
#include <netpacket/packet.h>
#include <net/ethernet.h>
#include <linux/fs.h>
#include "../include/cross_space.h"
#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "../include/ioctl_interface.h"


static int ioctl_sock, fp;
static struct ifreq ifr;

int wmpInit(char *ifname) {
	strcpy(ifr.ifr_name, ifname);

	if ((ioctl_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
		perror("Error: Unable to create the ioctl socket");
		return 0;
	}

	fp = open("/dev/rt-wmp",O_RDWR);
	if (fp < 0){
		fprintf(stderr, "Error: Unable to open char device '/dev/rt-wmp'\nHave you insmod-ed the module?\n");
		return 0;
	}
	return 1;
}

void wmpClose() {
	close(ioctl_sock);
}

int wmpPush(Msg *p) {
	return wmpPushData(0,p->data,p->len,p->dest,p->priority);
}

unsigned int wmpPop(Msg * p) {
	return wmpPopDataTimeoutCopy(0,p->data,&p->len, &p->src,&p->priority,0);
}

int wmpTimedPop(Msg * p, int timeout_ms) {
	return wmpPopDataTimeoutCopy(0,p->data,&p->len, &p->src,&p->priority,timeout_ms);
}

int wmpNonBlockingPop(Msg * p) {
	return wmpPopDataTimeoutCopy(0,p->data,&p->len, &p->src,&p->priority,-1);
}

int wmpPopDataCopy(unsigned int port, char  * p, unsigned int * size, unsigned char * src, signed char * priority){
	return wmpPopDataTimeoutCopy(port,p,size,src,priority,0);
}

//int wmpPushData(unsigned int port, char  * p, unsigned int   size, unsigned int   dest, char priority){
//	static int allocated = 0;
//	static char * buff = 0;
//	int len = size + sizeof(cross_space_data_t);
//	if (len > allocated) {
//		if (buff != 0) {
//			free(buff);
//		}
//		buff = malloc(len);
//		if (buff == 0) {
//			fprintf(stderr, "Unable to allocate memory (wmpPushData)");
//			return 1;
//		}else{
//			allocated = len;
//		}
//	}
//	cross_space_data_t * hdr = (cross_space_data_t *) buff;
//	hdr->dest = dest;
//	hdr->priority = priority;
//	hdr->size = size;
//	memcpy(buff + sizeof(cross_space_data_t), p, size);
//	write(fp, buff, len);
//	return 1;
//}
//
//int wmpPopDataTimeoutCopy (unsigned int port, char  * p, unsigned int * size, unsigned char * src, char * priority, int to){
//	int ret,ssize;
//	cross_space_data_t  * m = (cross_space_data_t *) p;
//	m->port = port;
//	m->timeout = 1000;
//	ret = read(fp, p, 1000);
//	if (ret >= 0){
//		*size = m->size;
//		*priority = m->priority;
//		*src = m->src;
//		ssize = m->size;
//		fprintf(stderr,"size: %d p:%d p+:%d %d\n",m->size,p,p+sizeof(cross_space_data_t),(size_t) ssize);
//		memmove(p,p+sizeof(cross_space_data_t),(size_t) ssize);
//		return 1;
//	}
//	return 0;
//}



int wmpPushData(unsigned int port, char  * p, unsigned int   size, unsigned int   dest, signed char priority){
	cross_space_data_t hdr ;
	hdr.dest = dest;
	hdr.priority = priority;
	hdr.size = size;
	hdr.port = port;
	write(fp, &hdr, 1);
	write(fp, p, 2);
	return 1;
}

int wmpPopDataTimeoutCopy (unsigned int port, char  * p, unsigned int * size, unsigned char * src, signed char * priority, int to){
	int ret;
	cross_space_data_t * hdr =(cross_space_data_t *) p;
	hdr->port = port;
	hdr->timeout = to;
	hdr->step = 0;
	fprintf(stderr, "first read port:%d\n",hdr->port);

	ret = read(fp, p, 1);
	fprintf(stderr, "first read done port:%d\n",hdr->port);
	if (ret>0){
		*size = hdr->size;
		*priority = hdr->priority;
		*src = hdr->src;
		ret = read(fp, p, 2);
		fprintf(stderr, "second read done ret:%d \n", ret);
	}
	if (ret >= 0){
		fprintf(stderr, "size: %d \n", hdr->size);
		fprintf(stderr, "return 1\n");
		return 1;
	}
	fprintf(stderr, "return 0\n");
	return 0;
}

int wmpPopData(unsigned int port, char  ** p, unsigned int * size, unsigned char * src, signed char * priority){
	return wmpPopDataTimeout(port,p,size,src,priority, 0);
}

int wmpPopDataTimeout(unsigned int port, char  ** p, unsigned int * size, unsigned char * src, signed char * priority, int to){
	int ret;
	char * q;
	cross_space_data_t hdr;
	hdr.port = port;
	hdr.timeout = to;
	hdr.step = 0;

	ret = read(fp, &hdr, 1);
	fprintf(stderr, "first read done port:%d\n",hdr.port);
	if (ret>0){
		*size = hdr.size;
		*priority = hdr.priority;
		*src = hdr.src;
		q = (char *) malloc(hdr.size);
		if (q==0){
			fprintf(stderr,"Error: Unable to allocate memory (wmpPopDataTimeout\n");
			return 0;
		}else{
			fprintf(stderr,"Allocate memory (%d bytes)\n",hdr.size);

		}
		memcpy(q,&hdr,sizeof(cross_space_data_t));
		ret = read(fp, q, 2);
		*p = q;
		return (int) q;
	}
	return 0;
}

void wmpPopDataDone(int id){
	char * p = (char*) id;
	if (p!=0){
		free (p);
	}
}

char wmpGetNodeId(void) {
	tpNodeInfo info;
	info.type = NODEID;

	ifr.ifr_data = (void *) &info;
	if ((ioctl(ioctl_sock, SIO_NODEINFO, &ifr)) < 0) {
		perror("Error in wmpGetNodeId IOCTL");
		exit(1);
	}
	return info.ret;
}

char wmpGetNumOfNodes(void) {
	tpNodeInfo info;
	info.type = NUMOFNODES;

	ifr.ifr_data = (void *) &info;
	if ((ioctl(ioctl_sock, SIO_NODEINFO, &ifr)) < 0) {
		perror("Error in wmpGetNumOfNodes IOCTL");
		exit(1);
	}
	return info.ret;
}

int wmpGetLatestLQM(char * lqm) {
	int size = wmpGetNumOfNodes();
	size *= size;

	ifr.ifr_data = (void *) lqm;
	if ((ioctl(ioctl_sock, SIO_GETLATESTLQM, &ifr)) < 0) {
		perror("Error in wmpGetLatestLQM IOCTL");
		exit(1);
	}
	return size;
}

int wmpIsNetworkConnected(void) {
	tpNetworkConnectedInfo info;
	info.type = NONBLOCKING;

	ifr.ifr_data = (void *) &info;
	if ((ioctl(ioctl_sock, SIO_NETWORKCONNECTED, &ifr)) < 0) {
		perror("Error in wmpIsNetworkConnected IOCTL");
		exit(1);
	}
	return info.ret;
}

int wmpIsNetworkConnectedBlocking(int timeout_ms) {
	tpNetworkConnectedInfo info;
	info.type = BLOCKING;
	info.timeout = timeout_ms;

	ifr.ifr_data = (void *) &info;
	if ((ioctl(ioctl_sock, SIO_NETWORKCONNECTED, &ifr)) < 0) {
		perror("Error in wmpIsNetworkConnectedBlocking IOCTL");
		exit(1);
	}
	return info.ret;
}

int wmp_queue_tx_remove_head(void) {
	tpQueueActionInfo info;
	info.queueAction = REMOVETXMSG;

	ifr.ifr_data = (void *) &info;
	if ((ioctl(ioctl_sock, SIO_QUEUEACTIONS, &ifr)) < 0) {
		perror("Error in wmp_queue_tx_remove_head IOCTL");
		exit(1);
	}
	return info.ret;
}

void wmpSetCpuDelay(int val) {
	tpQueueActionInfo info;
	info.queueAction = SETCPUDELAY;
	info.ival = val;

	ifr.ifr_data = (void *) &info;
	if ((ioctl(ioctl_sock, SIO_QUEUEACTIONS, &ifr)) < 0) {
		perror("Error in wmpSetCpuDelay IOCTL");
		exit(1);
	}
}

int wmpGetCpuDelay() {
	tpQueueActionInfo info;
	info.queueAction = GETCPUDELAY;

	ifr.ifr_data = (void *) &info;
	if ((ioctl(ioctl_sock, SIO_QUEUEACTIONS, &ifr)) < 0) {
		perror("Error in wmpGetCpuDelay IOCTL");
		exit(1);
	}
	return info.ival;
}

void wmpSetTimeout(int val) {
	tpQueueActionInfo info;
	info.queueAction = SETTIMEOUT;
	info.ival = val;

	ifr.ifr_data = (void *) &info;
	if ((ioctl(ioctl_sock, SIO_QUEUEACTIONS, &ifr)) < 0) {
		perror("Error in wmpSetTimeout IOCTL");
		exit(1);
	}
}

int wmpGetTimeout() {
	tpQueueActionInfo info;
	info.queueAction = GETTIMEOUT;

	ifr.ifr_data = (void *) &info;
	if ((ioctl(ioctl_sock, SIO_QUEUEACTIONS, &ifr)) < 0) {
		perror("Error in wmpGetTimeout IOCTL");
		exit(1);
	}
	return info.ival;
}

void wmpSetWCMult(int val) {
	tpQueueActionInfo info;
	info.queueAction = SETWCMULT;
	info.fval = val;

	ifr.ifr_data = (void *) &info;
	if ((ioctl(ioctl_sock, SIO_QUEUEACTIONS, &ifr)) < 0) {
		perror("Error in wmpSetWCMult IOCTL");
		exit(1);
	}
}

int wmpGetWCMult() {
	tpQueueActionInfo info;
	info.queueAction = GETWCMULT;

	ifr.ifr_data = (void *) &info;
	if ((ioctl(ioctl_sock, SIO_QUEUEACTIONS, &ifr)) < 0) {
		perror("Error in wmpGetWCMult IOCTL");
		exit(1);
	}
	return info.fval;
}

void wmpSetRate(int val) {
	tpQueueActionInfo info;
	info.queueAction = SETRATE;
	info.fval = val;

	ifr.ifr_data = (void *) &info;
	if ((ioctl(ioctl_sock, SIO_QUEUEACTIONS, &ifr)) < 0) {
		perror("Error in wmpSetRate IOCTL");
		exit(1);
	}
}

int wmpGetRate() {
	tpQueueActionInfo info;
	info.queueAction = GETRATE;

	ifr.ifr_data = (void *) &info;
	if ((ioctl(ioctl_sock, SIO_QUEUEACTIONS, &ifr)) < 0) {
		perror("Error in wmpGetRate IOCTL");
		exit(1);
	}
	return info.fval;
}

int wmp_queue_tx_get_room(void) {
	tpGetQueueElemsInfo info;
	info.type = NUMOFFREEPOSITIONSTX;

	ifr.ifr_data = (void *) &info;
	if ((ioctl(ioctl_sock, SIO_QUEUEELEMSINFO, &ifr)) < 0) {
		perror("Error in wmp_queue_tx_get_room IOCTL");
		exit(1);
	}
	return info.ret;
}

int wmpGetNumOfElementsInTXQueue(void) {
	tpGetQueueElemsInfo info;
	info.type = NUMOFELEMSTX;

	ifr.ifr_data = (void *) &info;
	if ((ioctl(ioctl_sock, SIO_QUEUEELEMSINFO, &ifr)) < 0) {
		perror("Error in wmpGetNumOfElementsInTXQueue IOCTL");
		exit(1);
	}
	return info.ret;
}

int wmpGetNumOfElementsInRXQueue(int port) {
	tpGetQueueElemsInfo info;
	info.type = NUMOFELEMSRX;

	ifr.ifr_data = (void *) &info;
	if ((ioctl(ioctl_sock, SIO_QUEUEELEMSINFO, &ifr)) < 0) {
		perror("Error in wmpGetNumOfElementsInRXQueue IOCTL");
		exit(1);
	}
	return info.ret;
}

int wmpGetNetIT(void) {
	tpRTWMPSetGetInfo info;
	info.type = NETIT;

	ifr.ifr_data = (void *) &info;
	if ((ioctl(ioctl_sock, SIO_RTWMPSETGET, &ifr)) < 0) {
		perror("Error in wmpGetNetIT IOCTL");
		exit(1);
	}
	return info.netIT;
}

unsigned int wmpGetMTU(void) {
	tpRTWMPSetGetInfo info;
	info.type = GETMTU;

	ifr.ifr_data = (void *) &info;
	if ((ioctl(ioctl_sock, SIO_RTWMPSETGET, &ifr)) < 0) {
		perror("Error in wmpGetMTU IOCTL");
		exit(1);
	}
	return info.mtu;
}

void wmpSetActiveSearch(int val) {
	tpRTWMPSetGetInfo info;
	info.type = SETAS;
	info.activeSearch = val;

	ifr.ifr_data = (void *) &info;
	if ((ioctl(ioctl_sock, SIO_RTWMPSETGET, &ifr)) < 0) {
		perror("Error in wmpSetActiveSearch IOCTL");
		exit(1);
	}
}

int wmpGetActiveSearch(void) {
	tpRTWMPSetGetInfo info;
	info.type = GETAS;

	ifr.ifr_data = (void *) &info;
	if ((ioctl(ioctl_sock, SIO_RTWMPSETGET, &ifr)) < 0) {
		perror("Error in wmpGetActiveSearch IOCTL");
		exit(1);
	}
	return info.activeSearch;
}

void wmpSetInstanceId(short iid) {
	tpRTWMPSetGetInfo info;
	info.type = SINSTANCEID;
	info.instanceId = iid;

	ifr.ifr_data = (void *) &info;
	if ((ioctl(ioctl_sock, SIO_RTWMPSETGET, &ifr)) < 0) {
		perror("Error in wmpSetInstanceId IOCTL");
		exit(1);
	}
}

short wmpGetInstanceId() {
	tpRTWMPSetGetInfo info;
	info.type = GINSTANCEID;

	ifr.ifr_data = (void *) &info;
	if ((ioctl(ioctl_sock, SIO_RTWMPSETGET, &ifr)) < 0) {
		perror("Error in wmpGetInstanceId IOCTL");
		exit(1);
	}
	return info.instanceId;
}

void wmpSetPrimBasedRouting(int val) {
	tpRTWMPSetGetInfo info;
	info.type = SPRIMBASEDROUTING;
	info.primBasedRouting = val;

	ifr.ifr_data = (void *) &info;
	if ((ioctl(ioctl_sock, SIO_RTWMPSETGET, &ifr)) < 0) {
		perror("Error in wmpSetPrimBasedRouting IOCTL");
		exit(1);
	}
}

int wmpGetPrimBasedRouting() {
	tpRTWMPSetGetInfo info;
	info.type = GPRIMBASEDROUTING;

	ifr.ifr_data = (void *) &info;
	if ((ioctl(ioctl_sock, SIO_RTWMPSETGET, &ifr)) < 0) {
		perror("Error in wmpGetPrimBasedRouting IOCTL");
		exit(1);
	}
	return info.primBasedRouting;
}

void wmpSetMessageReschedule(int val) {
	tpRTWMPSetGetInfo info;
	info.type = SMESSAGERESCHEDULE;
	info.messageReschedule = val;

	ifr.ifr_data = (void *) &info;
	if ((ioctl(ioctl_sock, SIO_RTWMPSETGET, &ifr)) < 0) {
		perror("Error in wmpSetMessageReschedule IOCTL");
		exit(1);
	}
}

int wmpGetMessageReschedule() {
	tpRTWMPSetGetInfo info;
	info.type = GMESSAGERESCHEDULE;

	ifr.ifr_data = (void *) &info;
	if ((ioctl(ioctl_sock, SIO_RTWMPSETGET, &ifr)) < 0) {
		perror("Error in wmpGetMessageReschedule IOCTL");
		exit(1);
	}
	return info.messageReschedule;
}

void wmpSetFlowControl(int val) {
	tpRTWMPSetGetInfo info;
	info.type = SFLOWCONTROL;
	info.flowControl = val;

	ifr.ifr_data = (void *) &info;
	if ((ioctl(ioctl_sock, SIO_RTWMPSETGET, &ifr)) < 0) {
		perror("Error in wmpSetFlowControl IOCTL");
		exit(1);
	}
}

int wmpGetFlowControl() {
	tpRTWMPSetGetInfo info;
	info.type = GFLOWCONTROL;

	ifr.ifr_data = (void *) &info;
	if ((ioctl(ioctl_sock, SIO_RTWMPSETGET, &ifr)) < 0) {
		perror("Error in wmpGetFlowControl IOCTL");
		exit(1);
	}
	return info.flowControl;
}

unsigned int wmpGetSerial(void) {
	tpRTWMPSetGetInfo info;
	info.type = SERIAL;

	ifr.ifr_data = (void *) &info;
	if ((ioctl(ioctl_sock, SIO_RTWMPSETGET, &ifr)) < 0) {
		perror("Error in wmpGetSerial IOCTL");
		exit(1);
	}
	return info.serial;
}

unsigned int wmpGetLoopId(void) {
	tpRTWMPSetGetInfo info;
	info.type = LOOPID;

	ifr.ifr_data = (void *) &info;
	if ((ioctl(ioctl_sock, SIO_RTWMPSETGET, &ifr)) < 0) {
		perror("Error in wmpGetLoopId IOCTL");
		exit(1);
	}
	return info.loopId;
}

int wmpSetup(char p1, char p2) {
	return wmpInit("wmp0");
}

void wmpRun() {
	while (1) {
		sleep(1);
	}
}

void wmpRunBG() {

}

int  wmpIsKernelSpace(){
	return 1;
}

char lqm_get_val(int i, int j){
	return 0;
}