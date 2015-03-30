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

#include <stdarg.h>
#include <sys/time.h>
#include <unistd.h>
#include <stdio.h>
#include <time.h>
#include <sys/socket.h>
#include <netinet/in.h>
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
#include <errno.h>
#include <ctype.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/if.h>
#include <netpacket/packet.h>
#include <net/ethernet.h>     /* the L2 protocols */
#include <asm/types.h>
#include <time.h>
#include <netinet/if_ether.h>
#include "radiotap.h"
#include "radiotap_iter.h"
#include "bridge.h"
#include "frames.h"
#include "timespec_utils.h"

#define WMP_MSG fprintf
#define MAX_PACKET_LEN 2342

static int tx, use_monitor = 1, rx, protocol;
static struct sockaddr_ll broadcast;
static struct L1Header * rxb_head, *txb_head;
static char txb[65535], rxb[65536], *txb_data, *rxb_data, mnet_size, mnode_id;
static unsigned char  *path;
static pthread_mutex_t mtx = PTHREAD_MUTEX_INITIALIZER;
static sem_t * sem;


unsigned char get_next(unsigned char dest) {
	unsigned char i, me, him;
	for (i = 0; i < mnet_size; i++) {
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

static void listener(void *ptr) {
	fprintf(stderr, "Listening...\n");
	while (1) {
		int rlen = recvfrom(rx, rxb, MAX_PACKET_LEN, 0, 0, 0);
		if (rlen == -1) {
			sleep(1);
			fprintf(stderr, "Discarding (error in RX)...\n");
			continue;
		}

		if (use_monitor) {
			struct ieee80211_radiotap_header * rte = (struct ieee80211_radiotap_header *) rxb;
			struct ieee80211_frame * eh = (struct ieee80211_frame *) (rxb + rte->it_len);
			if (eh->fcs != 8) {
				fprintf(stderr, "Discarding (NOT Data frame)...\n");
				continue;
			}
			struct llc * p = (struct llc *) (rxb + rte->it_len + sizeof(struct ieee80211_frame));
			if (p->proto != protocol) {
				fprintf(stderr, "Discarding (NOT expected protocol %x)...\n", protocol);
				continue;
			}
			parse_radiotap(rte, &pi);
			rxb_head = (struct L1Header *) (rxb + rte->it_len + sizeof(struct ieee80211_frame) + sizeof(struct llc));
			rxb_data = (char *) (rxb + rte->it_len + sizeof(struct ieee80211_frame) + sizeof(struct llc) + sizeof(struct L1Header));

			fprintf(stderr, " 802.11 type:%x protocol:%x\n", eh->fcs, p->proto);
		} else {
			struct ethhdr * eh = (struct ethhdr *) (rxb);
			rxb_head = (struct L1Header *) (rxb + ETH_HLEN);
			rxb_data = (char *) (rxb + ETH_HLEN + sizeof(struct L1Header));
			if (eh->h_proto != protocol) {
				fprintf(stderr, "Discarding (NOT expected protocol %x)...\n", protocol);
				continue;
			}
			pi.valid = 0;
		}
		fprintf(stderr, "Ok!");
		continue;

		fprintf(stderr, "Received, len:%d dest:%d to:%d me:%d\n", rlen, rxb_head->dest, rxb_head->to, mnode_id);
		if (rxb_head->from == mnode_id) {
			fprintf(stderr, "Mine, Discarding!\n");
		} else if (rxb_head->to != mnode_id) {
			fprintf(stderr, "Not for me, Discarding!\n");
		} else if (rxb_head->dest == mnode_id) {
			fprintf(stderr, "Mine!\n");
			pthread_mutex_lock(&mtx);
			rxb_head->len = rlen;
			sem_post(&sem[rxb_head->port]);
		} else if (rxb_head->to == mnode_id) {
			pthread_mutex_lock(&mtx);
			memcpy(txb + ETH_HLEN, rxb + ETH_HLEN, 65536 - ETH_HLEN);
			txb_head->to = get_next(txb_head->dest);
			txb_head->from = mnode_id;
			int res = sendto(tx, txb, rlen, 0, (struct sockaddr*) &broadcast, sizeof(broadcast));

			fprintf(stderr, "Forwarded!\n");
			fprintf(stderr, "Sent dest:%d to:%d port: %d len: %d\n", txb_head->dest, txb_head->to, txb_head->port, rlen);

			pthread_mutex_unlock(&mtx);
		}
	}
}


int L2_setup(int proto, int use_mon, unsigned char node_id, unsigned char net_size, int num_ports) {
	mnode_id = node_id;
	mnet_size = net_size;

	sem = (sem_t *) malloc(num_ports * sizeof(sem_t));
	path = (char *) malloc(net_size);

	int i;
	for (i = 0; i < num_ports; i++) {
		sem_init(&sem[i], 0, 0);
	}

	for (i = 0; i < mnet_size; i++) {
		path[i] = i;
	}

	read_config(".rwp", "rwp.hw");

	int res = L1_setup(proto, use_mon);

	pthread_t th;
	res |= pthread_create(&th, NULL, (void *) &listener, NULL);

	return res;
}

int L2_send(unsigned int port, char * buf, unsigned int len, unsigned int dest) {
	txb_head->dest = dest;
	txb_head->from = mnode_id;
	txb_head->src = mnode_id;
	txb_head->to = get_next(txb_head->dest);
	txb_head->port = port;
	txb_head->len = len;
	memcpy(txb_data, buf, len);
	int size = len + ETHER_HDR_LEN + sizeof(struct L1Header);
	int res = sendto(tx, txb, size, 0, (struct sockaddr*) &broadcast, sizeof(broadcast));

	fprintf(stderr, "Sent dest:%d to:%d port: %d len: %d\n", txb_head->dest, txb_head->to, txb_head->port, size);

	return (res == -1);
}

int wrp_receive_timeout(int to, unsigned int port, char ** buf, unsigned int * len, unsigned char * src, signed char * pri, struct WRPPacketInfo * info) {
	if (to != 0) {
		struct timespec ts;
		clock_gettime(CLOCK_REALTIME, &ts);
		timespec_addms(&ts, to);
		if (sem_timedwait(&sem[port], &ts) == ETIMEDOUT) {
			return ETIMEDOUT;
		}
	} else {
		sem_wait(&sem[port]);
	}

	if (info != 0) {
		memcpy(info, &pi, sizeof(struct WRPPacketInfo));
	}

	(*buf) = rxb_data;
	(*src) = rxb_head->src;
	(*pri) = rxb_head->priority;
	(*len) = rxb_head->len;
	return (*len) > 0 ? 0 : 1;
}

void wrp_receive_done() {
	pthread_mutex_unlock(&mtx);
}

int wrp_receive(unsigned int port, char ** p, unsigned int * size, unsigned char * src, signed char * priority, struct WRPPacketInfo * pi) {
	return wrp_receive_timeout(0, port, p, size, src, priority, pi);
}

int main() {
	setup(0x6969, 0, 5, 64, 0);
	while (1) {
		sleep(1);
	}
}

