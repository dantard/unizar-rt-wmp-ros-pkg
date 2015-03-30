/*
 * low_level.c
 *
 *  Created on: Jul 22, 2014
 *      Author: danilo
 */

/*------------------------------------------------------------------------
 *---------------------           RT-WMP              --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: ./src/core/interface/wmp_interface.h
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

#ifndef WMPINTERFACE__H
#define WMPINTERFACE__H

#include <stdarg.h>
#include "wmp_config.h"
#include "../rt-wmp/src/core/interface/Msg.h"
#include "../rt-wmp/src/core/include/frames.h"
#include <sys/time.h>
#include <unistd.h>
#include <stdio.h>
#include <time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <semaphore.h>
#include <sys/types.h>
#include <sys/time.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <string.h>
#include <sys/time.h>
#include <ctype.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <linux/if.h>
#include <netpacket/packet.h>
#include <net/ethernet.h>     /* the L2 protocols */
#include <asm/types.h>
#include <time.h>
#include <netinet/if_ether.h>

#define WMP_MSG fprintf

static struct sockaddr_ll tx_address;
static struct WRPHeader * rxb_header, *txb_header;
static char txb[65535], *txb_data, rxb[65536], *rxb_data, mactive_nodes, mnode_id;
static unsigned char src_mac[6], bcast_mac[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }, path[32];
static sem_t sem[64];
static pthread_mutex_t mtx = PTHREAD_MUTEX_INITIALIZER;
static int idx = 0,
rx, tx;

struct WRPHeader {
	unsigned char src;
	unsigned char dest;
	unsigned char from;
	unsigned char to;
	unsigned char port;
	unsigned int len;
	unsigned char priority;
};

unsigned char get_next(unsigned char dest) {
	unsigned char i, me, him;
	for (i = 0; i < mactive_nodes; i++) {
		fprintf(stderr, "Path[i] = %d mnode = %d dest = %d\n", path[i], mnode_id, dest);

		if (path[i] == mnode_id) {
			me = i;
		}
		if (path[i] == dest) {
			him = i;
		}
	}
	if (me > him) {
		return path[me - 1];
	} else {
		return path[me + 1];
	}
}

static int eth_raw_init(char * DEVICE, int tx) {
	int s;
	struct ifreq ifr;
	int ifindex = 0;

	s = socket(PF_PACKET, SOCK_RAW, htons(0x6868));
	if (s == -1) {
		perror("socket():");
		exit(1);
	}
	printf("Successfully opened socket: %i\n", s);

	strncpy(ifr.ifr_name, DEVICE, IFNAMSIZ);
	if (ioctl(s, SIOCGIFINDEX, &ifr) == -1) {
		perror("SIOCGIFINDEX");
		exit(1);
	}
	ifindex = ifr.ifr_ifindex;
	printf("Successfully got interface index: %i\n", ifindex);

	if (ioctl(s, SIOCGIFHWADDR, &ifr) == -1) {
		perror("SIOCGIFINDEX");
		exit(1);
	}

	if (tx) {
		tx_address.sll_family = PF_PACKET;
		tx_address.sll_protocol = htons(ETH_P_ALL);
		tx_address.sll_ifindex = ifindex;
		tx_address.sll_pkttype = PACKET_BROADCAST;
		tx_address.sll_halen = ETH_ALEN;
	}

	return s;
}

static int readllcfg() {
	char filename[256], line[256];
	char param[20], val[32];

	snprintf(filename, 256, "%s/.rt-wmp/rt-wmp-us-ip.ll", getenv("HOME"));

	FILE * f = fopen(filename, "r");
	if (f > 0) {
		WMP_MSG(stderr, "Reading Low Level Configuration file (%s)... \n", filename);
		while (fgets(line, 256, f) != NULL) {

			if (line[0] < 65 || line[0] > 90) {
				continue;
			}
			sscanf(line, "%s %s", param, val);
			int exists = 0;

			if (strcmp(param, "PATH") == 0) {
				int i;
				for (i = 0; i < strlen(val); i++) {
					path[i] = val[i] - '0';
				}
				exists = 1;
			}

			if (exists) {
				WMP_MSG(stderr, "READ OPTION: %s = %s\n", param, val);
			} else {
				WMP_MSG(stderr, "*** UKNOWN OPTION: %s = %s\n", param, val);
			}
		}
		WMP_MSG(stderr, "Done.\n");
	} else {
		WMP_MSG(stderr, "File %s not found, using default values\n", filename);
	}
	return 0;
}

char wmpGetNodeId(void) {
	return mnode_id;
}

char wmpGetNumOfNodes(void) {
	return mactive_nodes;
}

int wmpGetLatestLQM(char * lqm) {
	return 1;
}

int wmpIsNetworkConnected(void) {
	return 1;
}

void listener(void *ptr) {
	fprintf(stderr, "Listening...\n");
	while (1) {
		int rlen = recvfrom(rx, rxb, 2342, 0, 0, 0);
		fprintf(stderr, "Received, len:%d dest:%d to:%d me:%d\n", rlen, rxb_header->dest, rxb_header->to, mnode_id);
		if (rxb_header->from == mnode_id) {
			fprintf(stderr, "Mine, Discarding!\n");
		} else if (rxb_header->to != mnode_id) {
			fprintf(stderr, "Not for me, Discarding!\n");
		} else if (rxb_header->dest == mnode_id) {
			fprintf(stderr, "Mine!\n");
			pthread_mutex_lock(&mtx);
			rxb_header->len = rlen;
			sem_post(&sem[rxb_header->port]);
		} else if (rxb_header->to == mnode_id) {
			pthread_mutex_lock(&mtx);
			memcpy(txb + ETH_HLEN, rxb + ETH_HLEN, 65536 - ETH_HLEN);
			txb_header->to = get_next(txb_header->dest);
			txb_header->from = mnode_id;
			int res = sendto(tx, txb, rlen, 0, (struct sockaddr*) &tx_address, sizeof(tx_address));

			fprintf(stderr, "Forwarded!\n");
			fprintf(stderr, "Sent dest:%d to:%d port: %d len: %d\n", txb_header->dest, txb_header->to, txb_header->port, rlen);

			pthread_mutex_unlock(&mtx);
		}
	}
}

int wmpSetup(char node_id, char active_nodes) {
	mnode_id = node_id;
	mactive_nodes = active_nodes;

	int i;
	for (i = 0; i < 64; i++) {
		sem_init(&sem[i], 0, 0);
	}
	for (i = 0; i < mactive_nodes; i++) {
		path[i] = i;
	}

	readllcfg();

	/* RX */
	rxb_header = (struct WRPHeader *) (rxb + ETH_HLEN);
	rxb_data = (char *) (rxb + ETH_HLEN + sizeof(struct WRPHeader));

	/* TX */
	txb_header = (struct WRPHeader *) (txb + ETH_HLEN);
	txb_data = (char *) (txb + ETH_HLEN + sizeof(struct WRPHeader));
	struct ethhdr * eh = (struct ethhdr *) txb;
	eh->h_proto = htons(0x6868);
	memcpy((void *) eh->h_dest, (void*) bcast_mac, ETH_ALEN);
	memcpy((void *) eh->h_source, (void*) src_mac, ETH_ALEN);

	tx = eth_raw_init("wlan0", 1);
	rx = eth_raw_init("wlan0", 0);

	pthread_t th;
	pthread_create(&th, NULL, (void *) &listener, NULL);

	return 1;
}
void wmpRunBG(void) {
	return;
}

unsigned int wmpGetSerial(void) {
	return idx++;
}
unsigned int wmpGetLoopId(void) {
	return idx++;
}

int wmpPushData(unsigned int port, char * p, unsigned int size, unsigned int dest, signed char priority) {

	int ip = 0;
	while (dest >>= 1) {
		++ip;
	}
	txb_header->dest = ip;
	txb_header->from = mnode_id;
	txb_header->src = mnode_id;
	txb_header->to = get_next(txb_header->dest);
	txb_header->port = port;
	txb_header->len = size;
	memcpy(txb_data, p, size);
	int len = size + ETHER_HDR_LEN + sizeof(struct WRPHeader);
	int res = sendto(tx, txb, len, 0, (struct sockaddr*) &tx_address, sizeof(tx_address));

	fprintf(stderr, "Sent dest:%d to:%d port: %d len: %d\n", txb_header->dest, txb_header->to, txb_header->port, len);

	return 1;
}

int wmpPopData(unsigned int port, char ** p, unsigned int * size, unsigned char * src, signed char * priority) {

	fprintf(stderr, "Waiting on port: %d\n", port);
	sem_wait(&sem[port]);
	fprintf(stderr, "Received on port: %d\n", port);

	*p = rxb + ETH_HLEN + sizeof(struct WRPHeader);
	*src = rxb_header->src;
	*priority = rxb_header->priority;
	*size = rxb_header->len;
	return (*size) > 0 ? 1 : -1;
}

int wmpPopDataTimeout(unsigned int port, char ** p, unsigned int * size, unsigned char * src, signed char * priority, int to) {
	return 1;
}

void wmpPopDataDone(int id) {
	pthread_mutex_unlock(&mtx);
	return;
}

int wmpIsKernelSpace(void) {
	return 0;
}

int wmpGetLatestDistances(char * dist) {
	return 1;
}

char lqm_get_val(int i, int j) {
	return 1;
}

void wmpSetMessageCallback(void (*f)(wmpFrame *)) {
	return;
}

void wmpForceLQM(char * lqm) {
	return;
}

int wmpSetParam(const char * txt, int val) {
	return 1;
}

int wmpGetParam(const char * txt) {
	return 1;
}

void getTimedFilename(char * str_time) {
	return;
}

void wmpForcePath(char * p) {
	memcpy(path, p, mactive_nodes);
	return;
}

unsigned long long getRawActualTimeus() {
	struct timeval tv;
	gettimeofday(&tv, 0);
	unsigned long long tim1 = tv.tv_sec;
	tim1 *= 1000000;
	tim1 += tv.tv_usec;
	return tim1;
}

#endif

