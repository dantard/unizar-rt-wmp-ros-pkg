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
#include "types.h"

#define WMP_MSG fprintf
#define MAX_PACKET_LEN 2342

static int tx, use_monitor = 1, rx, protocol, rxb_data_len;
static struct sockaddr_ll broadcast;
static struct L1Header * rxb_head, *txb_head;
static char txb[65535], rxb[65536], *txb_data, *rxb_data;
static unsigned char bcast_mac[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

static int sock_raw_init(char * DEVICE, int protocol, int * sock, int * if_idx, unsigned char src_mac[6]) {
	int s, ifindex, i;
	struct ifreq ifr;

	s = socket(PF_PACKET, SOCK_RAW, htons(protocol));
	if (s == -1) {
		perror("socket():");
		return 1;
	}
	printf("Successfully opened socket: %i\n", s);

	strncpy(ifr.ifr_name, DEVICE, IFNAMSIZ);
	if (ioctl(s, SIOCGIFINDEX, &ifr) == -1) {
		perror("SIOCGIFINDEX");
		return 1;
	}
	ifindex = ifr.ifr_ifindex;
	printf("Successfully got interface index: %i\n", ifindex);

	if (ioctl(s, SIOCGIFHWADDR, &ifr) == -1) {
		perror("SIOCGIFINDEX");
		return 1;
	}

	for (i = 0; i < 6; i++) {
		src_mac[i] = ifr.ifr_hwaddr.sa_data[i];
	}

	/* bind socket to interface to receive frame ONLY from that interface */
	struct sockaddr_ll sll;
	sll.sll_family = AF_PACKET;
	sll.sll_ifindex = ifindex;
	sll.sll_protocol = htons(protocol);
	if ((bind(s, (struct sockaddr *) &sll, sizeof(sll))) == -1) {
		perror("bind: ");
		return 1;
	}

	ifr.ifr_flags |= IFF_PROMISC;
	if (ioctl(s, SIOCSIFFLAGS, &ifr) == -1) {
		perror("Error: Could not set flag IFF_PROMISC");
		return 1;
	}

	(*sock) = s;
	(*if_idx) = ifindex;
	return 0;
}

static int read_config(char * directory, char * fname) {
	char filename[256], line[256];
	char param[20], val[32];

	snprintf(filename, 256, "%s/%s/%s", getenv("HOME"), directory, fname);

	FILE * f = fopen(filename, "r");
	if (f > 0) {
		WMP_MSG(stderr, "Reading configuration file (%s)... \n", filename);
		while (fgets(line, 256, f) != NULL) {

			if (line[0] < 65 || line[0] > 90) {
				continue;
			}
			sscanf(line, "%s %s", param, val);
			int exists = 0;

			if (strcmp(param, "L1:PATH") == 0) {
				int i;
				for (i = 0; i < strlen(val); i++) {

				}
				exists = 1;
			}

			if (exists) {
				WMP_MSG(stderr, "READ OPTION: %s = %s\n", param, val);
			}
		}
		WMP_MSG(stderr, "Done.\n");
		fclose(f);
	} else {
		WMP_MSG(stderr, "File %s not found, using default values\n", filename);
	}
	return 0;
}

static void parse_radiotap(struct ieee80211_radiotap_header * buf, struct WRPPacketInfo * pi) {
	int pkt_rate_100kHz = 0, antenna = 0, pwr = 0;
	char rssi_dbm = 0, noise_dbm = 0;
	struct ieee80211_radiotap_iterator iterator;

	int ret = ieee80211_radiotap_iterator_init(&iterator, buf, buf->it_len, 0);
	WMP_MSG(stderr, "ret1: %d buf->it_len:%d \n", ret, buf->it_len);
	while (!ret) {
		ret = ieee80211_radiotap_iterator_next(&iterator);
		WMP_MSG(stderr, "it\n");
		if (ret) {
			WMP_MSG(stderr, "it exit...\n");
			continue;
		}
		switch (iterator.this_arg_index) {
		case IEEE80211_RADIOTAP_RATE:
			pi->rate = (*iterator.this_arg) * 5;
			break;
		case IEEE80211_RADIOTAP_DBM_ANTSIGNAL:
			pi->rssi = (*iterator.this_arg);
			break;
		case IEEE80211_RADIOTAP_CHANNEL:
			pi->channel = (*iterator.this_arg);
			break;
		case IEEE80211_RADIOTAP_ANTENNA:
			pi->antenna = (*iterator.this_arg);
			break;
		case IEEE80211_RADIOTAP_DBM_TX_POWER:
			pi->pwr = *iterator.this_arg;
			break;
		case IEEE80211_RADIOTAP_DB_ANTNOISE:
			pi->noise_dbm = *iterator.this_arg;
			break;
		default:
			break;
		}
		pi->valid = 1;
	}
}


int L1_receive(int timeout, char * buf, unsigned char * from, unsigned char * to, struct WRPPacketInfo * pi) {

	int orig_timeout = timeout, elapsed = 0, rlen;
	struct timespec tic, tac;
	timespec_now(&tic);

	pi->foreign = 0;
	while (1) {
		if (orig_timeout == 0) {
			rlen = recvfrom(rx, rxb, MAX_PACKET_LEN, 0, 0, 0);
		} else {
			struct timeval tv;
			fd_set fd_rx;
			elapsed = timespec_elapsed_ms(&tic);
			timeout = orig_timeout - elapsed;
			tv.tv_sec = 0;
			tv.tv_usec = 1000 * timeout;
			FD_ZERO(&fd_rx);
			FD_SET(rx, &fd_rx);
			int r = select(FD_SETSIZE, &fd_rx, NULL, NULL, &tv);
			if (r) {
				rlen = recvfrom(rx, rxb, MAX_PACKET_LEN, 0, 0, 0);
			} else {
				pi->delay = timespec_elapsed_ms(&tic);
				return ETIMEDOUT;
			}
		}

		if (use_monitor) {
			struct ieee80211_radiotap_header * rte = (struct ieee80211_radiotap_header *) rxb;
			struct ieee80211_frame * eh = (struct ieee80211_frame *) (rxb + rte->it_len);
			if (eh->fcs != 8) {
				fprintf(stderr, "Discarding (NOT Data frame)...\n");
				pi->foreign++;
				continue;
			}
			struct llc * p = (struct llc *) (rxb + rte->it_len + sizeof(struct ieee80211_frame));
			if (p->proto != protocol) {
				fprintf(stderr, "Discarding (NOT expected protocol %x)...\n", protocol);
				pi->foreign++;
				continue;
			}

			parse_radiotap(rte, pi);

			rxb_head = (struct L1Header *) (rxb + rte->it_len + sizeof(struct ieee80211_frame) + sizeof(struct llc));
			rxb_data = (char *) (rxb + rte->it_len + sizeof(struct ieee80211_frame) + sizeof(struct llc) + sizeof(struct L1Header));
			rxb_data_len = rlen - (rte->it_len + sizeof(struct ieee80211_frame) + sizeof(struct llc) + sizeof(struct L1Header));
			fprintf(stderr, " 802.11 type:%x protocol:%x\n", eh->fcs, p->proto);
		} else {
			struct ethhdr * eh = (struct ethhdr *) (rxb);
			rxb_head = (struct L1Header *) (rxb + ETH_HLEN);
			rxb_data = (char *) (rxb + ETH_HLEN + sizeof(struct L1Header));
			rxb_data_len  = rlen - (ETH_HLEN + sizeof(struct L1Header));
			if (eh->h_proto != protocol) {
				fprintf(stderr, "Discarding (NOT expected protocol %x)...\n", protocol);
				continue;
			}
		}
		break;
	}

	(*to) = rxb_head->to;
	(*from) = rxb_head->from;
	memcpy(buf, rxb_data, rxb_data_len);
	pi->delay = timespec_elapsed_ms(&tic);

	return 0;
}

int L1_setup(int proto, int use_mon) {
	protocol = proto;
	use_monitor = use_mon;

	read_config(".rwp", "rwp.hw");

	int txi, rxi, res = 0;
	unsigned char rx_mac[6], tx_mac[6];

	if (use_monitor) {
		delete_interface("mon0");
		create_monitor("wlan0", "mon0");
		set_ip("mon0", 0);
		res |= sock_raw_init("mon0", ETH_P_ALL, &rx, &rxi, rx_mac);
	} else {
		res |= sock_raw_init("wlan0", ETH_P_ALL, &rx, &rxi, rx_mac);
	}

	res |= sock_raw_init("wlan0", protocol, &tx, &txi, tx_mac);

	/* TX */
	txb_head = (struct L1Header *) (txb + ETH_HLEN);
	txb_data = (char *) (txb + ETH_HLEN + sizeof(struct L1Header));
	struct ethhdr * eh = (struct ethhdr *) txb;
	eh->h_proto = htons(protocol);
	memcpy((void *) eh->h_dest, (void*) bcast_mac, ETH_ALEN);
	memcpy((void *) eh->h_source, (void*) tx_mac, ETH_ALEN);

	broadcast.sll_family = PF_PACKET;
	broadcast.sll_protocol = htons(protocol);
	broadcast.sll_ifindex = txi;
	broadcast.sll_pkttype = PACKET_BROADCAST;
	broadcast.sll_halen = ETH_ALEN;

	return res;
}

int L1_send(char * buf, unsigned char from, unsigned char to, unsigned int len) {
	txb_head->from = from;
	txb_head->to = to;
	txb_head->len = len;
	memcpy(txb_data, buf, len);
	int size = len + ETHER_HDR_LEN + sizeof(struct L1Header);
	int res = sendto(tx, txb, size, 0, (struct sockaddr*) &broadcast, sizeof(broadcast));

	fprintf(stderr, "Sent to:%d len: %d\n", txb_head->to, txb_head->len);

	return (res == -1);
}

int main() {
	L1_setup(0x6969, 0, 5, 64, 1);
	char kk[1000];
	struct WRPPacketInfo pi;
	unsigned char a, b;
	while (1) {
		int res = L1_receive(10000, kk, &a, &b, &pi);

		fprintf(stderr, "Res:%d Delayed:%d foreign:%d rssi: %d \n", res, pi.delay, pi.foreign, pi.rssi);
	}
	while (1) {
		sleep(1);
	}
}

