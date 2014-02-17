/*------------------------------------------------------------------------
 *---------------------           RT-WMP              --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: ./src/platforms/linux_us/hwi/raw/ll_com.c
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
#include <pcap.h>
#include <netinet/if_ether.h>
#include <netinet/if_ether.h>

#include "core/include/ml_com.h"
#include "core/include/definitions.h"
#include "core/interface/wmp_interface.h"
#include "core/include/wmp_misc.h"
#include "core/include/frames.h"
#include "core/include/ll_com.h"

#include "radiotap/radiotap.h"
#include "radiotap/radiotap_iter.h"
#include <sys/ipc.h>
#include <sys/shm.h>
#include <fcntl.h>
#include "core/interface/wmp_interface.h"

static	sem_t * sem_tx, * sem_rx, * sem_ack;

static int s, rx, freq, txpower, dl, use_mon = 1, use_coord = 0;
static struct ethhdr *eh;
static struct sockaddr_ll socket_address;
static char DEV[20], ESSID[32], param[20], val[20], buffer[2500], *eth_head,
		*eth_data;
static unsigned char src_mac[6], bcast_mac[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF };
static struct pcap_pkthdr header;
static pcap_t *handle;
static int use_lo = 0;

int shmid;
key_t key;
char *shm;

void initshmem() {
	fprintf(stderr,"Preparing SHMEM\n");
	key = 5678;
	shmid = shmget(key, 2000, IPC_CREAT | 0666);
	if (shmid < 0) {
		perror("Failure in shmget");
		exit(-1);
	}
	shm = shmat(shmid, NULL, 0);
}

char * getPoint(){
	return shm;
}

static int readllcfg() {
	char filename[256], line[256];
	sprintf(DEV, "lo");
	sprintf(ESSID, "rt-wmp");
	freq = 2412;
	txpower = 15;

	snprintf(filename, 256, "%s/.rt-wmp/rt-wmp-us-raw.ll", getenv("HOME"));
	FILE * f = fopen(filename, "r");
	if (f > 0) {
		WMP_MSG(stderr, "Reading Low Level Configuration file (%s)... \n", filename);
		while (fgets(line, 256, f) != NULL) {

			if (line[0] < 65 || line[0] > 90) {
				continue;
			}
			sscanf(line, "%s %s", param, val);

			int exists = 0;
			if (strcmp(param, "DEVICE") == 0) {
				strcpy(DEV, val);
				exists = 1;
			} else if (strcmp(param, "FREQ") == 0) {
				freq = atoi(val);
				exists = 1;
			} else if (strcmp(param, "ESSID") == 0) {
				freq = atoi(val);
				exists = 1;
			} else if (strcmp(param, "TXPOWER") == 0) {
				txpower = atoi(val);
				exists = 1;
			} else if (strcmp(param, "USE_MONITOR") == 0) {
				use_mon = atoi(val);
				exists = 1;
			} else if (strcmp(param, "USE_COORDINATOR") == 0) {
				use_coord = atoi(val);
				if (use_coord){
					strcpy(DEV, "lo");
					use_mon = 0;
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

static void parse_radiotap(struct ieee80211_radiotap_header * buf, int * rate,
		char * rssi, char * noise) {
	int pkt_rate_100kHz = 0, antenna = 0, pwr = 0;
	char rssi_dbm = 0, noise_dbm = 0;
	struct ieee80211_radiotap_iterator iterator;
	int ret = ieee80211_radiotap_iterator_init(&iterator, buf, buf->it_len, 0);

	while (!ret) {

		ret = ieee80211_radiotap_iterator_next(&iterator);

		if (ret)
			continue;

		/* see if this argument is something we can use */

		switch (iterator.this_arg_index) {
		/*
		 * You must take care when dereferencing iterator.this_arg
		 * for multibyte types... the pointer is not aligned.  Use
		 * get_unaligned((type *)iterator.this_arg) to dereference
		 * iterator.this_arg for type "type" safely on all arches.
		 */
		case IEEE80211_RADIOTAP_RATE:
			/* radiotap "rate" u8 is in
			 * 500kbps units, eg, 0x02=1Mbps
			 */
			pkt_rate_100kHz = (*iterator.this_arg) * 5;
			break;
		case IEEE80211_RADIOTAP_DBM_ANTSIGNAL:
			rssi_dbm = (*iterator.this_arg);
			break;
		case IEEE80211_RADIOTAP_ANTENNA:
			/* radiotap uses 0 for 1st ant */
			antenna = (*iterator.this_arg);
			break;

		case IEEE80211_RADIOTAP_DBM_TX_POWER:
			pwr = *iterator.this_arg;
			break;
		case IEEE80211_RADIOTAP_DB_ANTNOISE:
			noise_dbm = *iterator.this_arg;
			break;
		default:
			break;
		}
	} /* while more rt headers */
	*noise = noise_dbm;
	*rssi = rssi_dbm;
	*rate = pkt_rate_100kHz;
}

static int pcap_init(char * dev, int promisc) {
	char errbuf[PCAP_ERRBUF_SIZE];
	handle = pcap_open_live(dev, BUFSIZ, promisc, 1000, errbuf);

	if (handle == NULL) {
		fprintf(stderr, "Couldn't open device %s: %s\n", dev, errbuf);
		return 0;
	}
	dl = pcap_datalink(handle);
	return 1;
}

static rxInfo pcap_sniff_packet(char * data, int delay_ms) {
	rxInfo rxi;
	int offset = 0, i, rate;
	char rssi, noise;
	struct timeval tv;
	const u_char *packet;

	tv.tv_sec = 0;
	tv.tv_usec = 1000 * delay_ms;

	fd_set set1;
	FD_ZERO(&set1);
	FD_SET(pcap_fileno(handle), &set1);
	select(FD_SETSIZE, &set1, NULL, NULL, &tv);

	if (FD_ISSET(pcap_fileno(handle), &set1)) {
		packet = pcap_next(handle, &header);
	} else {
		rxi.error = 1;
		return rxi;
	}

	if (packet == NULL) {
		rxi.error = 1;
		return rxi;
	}

	if (use_lo){
		usleep(10000); //to allow other threads to enter in the simulation
	}

	switch (dl) {
	case 1:
		offset = sizeof(struct ethhdr);
		break;

	case 127: {
		struct ieee80211_radiotap_header * rth =
				(struct ieee80211_radiotap_header*) packet;
		offset = rth->it_len + 24 + 8; //radiotap+80211
		parse_radiotap(rth, &rate, &rssi, &noise);
	}
		break;
	default:
		offset = 0;
	};

	if (header.len > offset) {
		short protocol_type = ntohs(*(short*) (packet + offset - 2));
		char * f_data = (char*) &packet[offset];
		int f_len = header.len - offset;
		f_len > 1500 ? 1500 : f_len;
		rxi.rssi = 96 + rssi;
		rxi.rssi = rxi.rssi > 100 ? 100 : rxi.rssi;
		rxi.rssi = rxi.rssi > 0 ? rxi.rssi : 1;
		rxi.rate = rate;
		rxi.noise = noise;
		rxi.proto = protocol_type;
		rxi.error = 0;
		rxi.size = f_len;
		rxi.has_lq = 1;
		memcpy(data, f_data, f_len);
		return rxi;
	} else {
		rxi.proto = 0;
		rxi.size = 0;
		rxi.error = 0;
		rxi.has_lq = 0;
		return rxi;
	}
}

static int eth_raw_init(char * DEVICE) {

	/* Vars */
	int i;
	struct ifreq ifr;
	int ifindex = 0; /*Ethernet Interface index*/

	/* Open socket */
	s = socket(PF_PACKET, SOCK_RAW, htons(WMP_TYPE_FIELD));
	if (s == -1) {
		perror("socket():");
		exit(1);
	}
	printf("Successfully opened socket: %i\n", s);

	/*Get ethernet interface index*/
	strncpy(ifr.ifr_name, DEVICE, IFNAMSIZ);
	if (ioctl(s, SIOCGIFINDEX, &ifr) == -1) {
		perror("SIOCGIFINDEX");
		exit(1);
	}
	ifindex = ifr.ifr_ifindex;
	printf("Successfully got interface index: %i\n", ifindex);

	/*retrieve corresponding MAC*/
	if (ioctl(s, SIOCGIFHWADDR, &ifr) == -1) {
		perror("SIOCGIFINDEX");
		exit(1);
	}
	for (i = 0; i < 6; i++) {
		src_mac[i] = ifr.ifr_hwaddr.sa_data[i];
	}
	WMP_MSG(stderr,"Host MAC address: %02X:%02X:%02X:%02X:%02X:%02X\n", src_mac[0],
			src_mac[1], src_mac[2], src_mac[3], src_mac[4], src_mac[5]);

	/*prepare sockaddr_ll*/
	socket_address.sll_family = PF_PACKET;
	socket_address.sll_protocol = htons(ETH_P_ALL);
	socket_address.sll_ifindex = ifindex;
	socket_address.sll_pkttype = PACKET_BROADCAST;
	socket_address.sll_halen = ETH_ALEN;
	if (strcmp(DEV, "lo") == 0) {
		WMP_MSG(stderr,"*** WARNING: Using 'lo' interface\n");
		for (i = 0; i < 6; i++) {
			socket_address.sll_addr[0] = src_mac[i];
		}
	} else {
		for (i = 0; i < 6; i++) {
			socket_address.sll_addr[0] = bcast_mac[i];
		}
	}
	socket_address.sll_addr[6] = 0x00;
	socket_address.sll_addr[7] = 0x00;
	return 0;
}

void closeLowLevelCom() {
	pcap_close(handle);
}

int initLowLevelCom() {

	char cmd[256];
	readllcfg();
	eth_raw_init(DEV);
	eth_head = buffer; /* eth_head points to beginning of buffer (ethernet header fields) */
	eth_data = buffer + ETH_HLEN; /* eth_data points to data field of the ethernet frame */
	eh = (struct ethhdr *) eth_head;
	use_lo = strcmp(DEV, "lo") == 0;

	if (use_coord) {
		initshmem();
		char name[64];
		sprintf(name, "sem_rx_%d", wmpGetNodeId());
		sem_rx = sem_open(name, O_CREAT, S_IRUSR | S_IWUSR, 0);
		sem_tx = sem_open("sem_tx", O_CREAT, S_IRUSR | S_IWUSR, 0);
		sem_ack = sem_open("sem_ack", O_CREAT, S_IRUSR | S_IWUSR, 0);
		return 1;
	}

	if (!use_lo && use_mon) {
		fprintf(stderr, "Checking sudo...");
		int res = system("sudo ls >/dev/null 2>1");
		if (res != 0){
			fprintf(stderr, "\nUnable to execute 'sudo' exiting...");
			exit(0);	
		}
		fprintf(stderr,"OK\n");
		usleep(100000);

		fprintf(stderr, "Checking iw...");
		res = system("sudo iw >/dev/null 2>1");
		if (res != 0){
			fprintf(stderr, "\nUnable to execute 'iw' exiting...");
			exit(0);	
		}
		fprintf(stderr,"OK\n");
		usleep(100000);
	
		fprintf(stderr, "Checking iwconfig...");
		res = system("sudo iwconfig >/dev/null 2>1");
		if (res != 0){
			fprintf(stderr, "\nUnable to execute 'iwconfig' exiting...");
			exit(0);	
		}
		fprintf(stderr,"OK\n");
		usleep(100000);

		fprintf(stderr, "Checking interface mon0...");
		res = system("sudo iw dev mon0 info >/dev/null 2>1");
		if (res == 0){
			fprintf(stderr, "OK\n");
		}else{
			fprintf(stderr, "does not exist, creating...");

			sprintf(cmd, "sudo ifconfig %s down >/dev/null 2>1", DEV);
			res = system(cmd);

			sprintf(cmd,"sudo iw dev %s interface add mon0 type monitor 2>/dev/null", DEV);
			res = system(cmd);
			if (res != 0){
				fprintf(stderr, "\nUnable to create mon0 interface exiting...");
				exit(0);	
			}else{
				fprintf(stderr,"OK\n");
			}
		}		
		usleep(500000);

		res = system("sudo ifconfig mon0 down >/dev/null 2>1");
		if (res != 0){
			fprintf(stderr, "Unable to put mon0 down, troubles ahead...\n");
		}
		usleep(100000);

		fprintf(stderr, "Setting frequency for mon0...");
		sprintf(cmd,"sudo iwconfig mon0 freq %dM  >/dev/null 2>1", freq);
		res = system(cmd);
		if (res != 0){
			fprintf(stderr, "failed (normal with some driver)\n");
		}else{
			fprintf(stderr,"OK\n");
		}
		usleep(100000);

		fprintf(stderr, "Setting mode ad-hoc for %s...", DEV);
		sprintf(cmd,"sudo iwconfig %s mode ad-hoc essid %s  >/dev/null 2>1", DEV, ESSID);
		res = system(cmd);
		if (res != 0){
			fprintf(stderr, "failed, exiting...\n");
			exit(0);
		}else{
			fprintf(stderr,"OK\n");
		}
		usleep(100000);

		fprintf(stderr, "Setting frequency and tx power for %s...", DEV);
		sprintf(cmd,"sudo iwconfig %s freq %dM txpower %d >/dev/null 2>1",DEV, freq, txpower);
		res = system(cmd);
		if (res != 0){
			fprintf(stderr, "failed\n");
		}else{
			fprintf(stderr,"OK\n");
		}
		usleep(100000);

		sprintf(cmd, "sudo ifconfig %s up 2>/dev/null", DEV);
		res = system(cmd);
		if (res != 0){
			fprintf(stderr, "Unable to put %s up, troubles ahead...\n",DEV);
		}
		usleep(100000);

		sprintf(cmd, "sudo ifconfig mon0 up 2>/dev/null", DEV);
		res = system(cmd);
		if (res != 0){
			fprintf(stderr, "Unable to put mon0 up, troubles ahead...\n");
		}
		usleep(100000);

		pcap_init("mon0", 1);
	} else {
		pcap_init(DEV, 0);
	}

	return 1;
}

static int llpsend(char * f, int size, int proto) {
	int res;
	memcpy((void *) eh->h_dest, (void*) bcast_mac, ETH_ALEN);
	memcpy((void *) eh->h_source, (void*) src_mac, ETH_ALEN);
	eh->h_proto = htons(proto);
	memcpy(eth_data, f, size);

	res = sendto(s, buffer, size + ETHER_HDR_LEN, 0,
			(struct sockaddr*) &socket_address, sizeof(socket_address));

	if (res == -1) {
		perror("sendto():");
		exit(1);
	}
	return res;
}

int llsend(char * f, int size) {
	if (use_coord){
		//fprintf(stderr,"Node %d waiting auth to tx\n",wmpGetNodeId());
		sem_wait(sem_tx);
		memcpy(shm,f,1500);
		sem_post(sem_ack);
		//fprintf(stderr,"Node %d txd\n",wmpGetNodeId());
		return 1;
	}else{
		return llpsend(f, size, WMP_TYPE_FIELD);
	}
}


char getSimulatedRssiRX(char * f){
	wmpFrame * p = (wmpFrame *) f;
	char from = p->hdr.from;
	char myself = wmpGetNodeId();
	if (from == 0){
		if (myself == 1) return 50;
		if (myself == 2) return 80;
	}
	if (from == 1){
		if (myself == 0) return 50;
		if (myself == 2) return 25;
	}
	if (from == 2){
		if (myself == 0) return 80;
		if (myself == 1) return 25;
	}
	return 80;
}


rxInfo llreceive(char *f, int timeout) {
	int r = 0;
	rxInfo ret;
	if (use_coord){
		sem_wait(sem_rx);
		memcpy(f,shm,1500);
		ret.size = 1500;
		ret.error = 0;
		ret.proto = 0x6969;
		ret.rate = 10;
		ret.has_lq = 1;
		ret.rssi = f[0];
		sem_post(sem_ack);
		return ret;
	}

	struct timeval tv;

	Token_Hdr * hh = (Token_Hdr * ) f;


	if (use_coord){
		//fprintf(stderr,"Node %d waiting rx auth\n", wmpGetNodeId());
		//fprintf(stderr,"Node %d wait rx type: %d\n", wmpGetNodeId(),hh->type);
		sem_wait(sem_rx);
	}


	if (!use_lo && use_mon) {
		return pcap_sniff_packet(f, timeout);
	} else {
		if (timeout > 0) {
			fd_set fd_rx;
			tv.tv_sec = 0;
			tv.tv_usec = 1000 * timeout;
			FD_ZERO(&fd_rx);
			FD_SET(s, &fd_rx);
			r = select(FD_SETSIZE, &fd_rx, NULL, NULL, &tv);
		} else {
			r = 1;
		}

//		if (r  && rand()%5 ==0){
//			r = 0;
//		}

		if (r) {
			int rlen = recvfrom(s, buffer, MTU, 0, 0, 0);
			ret.proto = ntohs(eh->h_proto);
			memcpy(f, buffer + ETHER_HDR_LEN, rlen - ETHER_HDR_LEN);

			ret.size = rlen - ETHER_HDR_LEN;
			ret.error = 0;
			ret.rate = 10;
			ret.has_lq = 0;
			int rssi = f[0];
			rssi = rssi*125/100;
			rssi = rssi>99?99:rssi;
			rssi = rssi<1?1:rssi;
			f[0] = (char) rssi;
		} else {
			ret.error = 1;
			return ret;
		}

		if (use_coord){
//			if (ret.error==1){
//				//fprintf(stderr,"Reposting sem_rx\n");
//				sem_post(sem_rx);
//			}else{
				//fprintf(stderr,"Node %d posting rx ack (type:%d)\n", wmpGetNodeId(), hh->type);
				sem_post(sem_ack);
//			}
		}

		return ret;
	}
}


