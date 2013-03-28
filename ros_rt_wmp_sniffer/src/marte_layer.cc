/*------------------------------------------------------------------------
 *---------------------           WMPSNIFFER          --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: marte_layer.cc
 *  Authors: Danilo Tardioli
 *  ----------------------------------------------------------------------
 *  Copyright (C) 2000-2012, Universidad de Zaragoza, SPAIN
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

#include <cstdio>
#include <unistd.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/ether.h>
#include <sys/utsname.h>
#include <net/if.h>
#include <fcntl.h>
#include <linux/if_packet.h>
#include <linux/if_ether.h>
#include <time.h>
#include <cstring>
#include <signal.h>
#include <cstdlib>
#include <map>
#include "marte_layer.h"
#if 0
#define DEBUG(x,args...) printf("%s: " x, __func__ , ##args)
#else
#define DEBUG(x,args...)
#endif

#define LOGGER_PROTOCOL 0x1010
#define OUTPUT_FILE "output_file.log"
#define NIC_INTERFACE "eth1"
/**
 * ETH_MIN_PAYLOAD is the minimum payload in an ethernet frame, if one sends
 *  less bytes the rest of the frame will be padded with zeros. To know when this
 *  situation happens we have added a flag in the first byte of each frame:
 *       - 0: if there is no padding (we sent data >= the 45)
 *       - 1-44: if there is padding (we sent data < 45)
 **/
#define ETH_MIN_PAYLOAD 46

typedef struct {
	struct ethhdr header;
	unsigned char less_than_min;
	unsigned char data[ETH_DATA_LEN - 1 ];
} __attribute__((__packed__)) eth_frame_t;

static int sock;
static bool marte_keep_running;
static int pfd[2];

void marte_layer_close() {
		fprintf(stdout,"Finishing the logger ");
		marte_keep_running = false;
		/* to unlock the select */
		char dsblq=0;
		if (write(pfd[1],&dsblq,1)==-1){
			fprintf(stdout,"Problem closing logger");
		}
}

int err;
char device[]= "eth1";//NIC_INTERFACE;
struct sockaddr_ll host_addr;
struct ifreq ifr;
ssize_t nbytes;
size_t written_bytes;
eth_frame_t receive_buffer;

int marte_layer_init(int nnodes) {
	fprintf(stderr,"Setting up MaRTE socket\n");
	sock=socket(PF_PACKET,SOCK_RAW,htons(LOGGER_PROTOCOL));
	if (sock < 0){
		fprintf(stderr,"Error Creating raw socket\n");
		return -1;
	}
	memset(&ifr, 0, sizeof(struct ifreq));
	fprintf(stderr,"Marte Sniffer Using interface %s\n",device);

	strcpy(ifr.ifr_name, device);
	err = ioctl(sock, SIOCGIFHWADDR, &ifr);
	if (err < 0){
		fprintf(stderr,"Error SIOCGIFHWADDR\n");
		return -1;
	}
	memcpy(host_addr.sll_addr, ifr.ifr_addr.sa_data, ETH_ALEN);
	err = ioctl(sock, SIOGIFINDEX, &ifr);
	if (err < 0) {
		fprintf(stderr,"Error SIOGIFINDEX\n");
		return -1;
	}
	host_addr.sll_ifindex = ifr.ifr_ifindex;
	host_addr.sll_family = AF_PACKET;
	host_addr.sll_protocol = htons(LOGGER_PROTOCOL);
	host_addr.sll_halen = ETH_ALEN;
	bind(sock, (struct sockaddr *)&host_addr, sizeof(host_addr));

	marte_keep_running = true;
	if (pipe(pfd) == -1) {
		perror("pipe");
		exit(EXIT_FAILURE);
	}
	return 1;
}

int marte_sniff_packet(char * data, simData_Hdr & sd, unsigned long long &time_us, std::map<int,robo_pose_t> & poses) {
	while (marte_keep_running) {

		fd_set set1;
		FD_ZERO(&set1);
		FD_SET(sock, &set1);
		FD_SET(pfd[0], &set1);
		select(pfd[0] + 1, &set1, NULL, NULL, 0);
		if (FD_ISSET(sock, &set1) ) {
			nbytes = recvfrom(sock, &receive_buffer, sizeof(receive_buffer), 0,	NULL, 0);
			if (nbytes < 0){
				fprintf(stderr," *** WARNING: Error reading from interface %s\n",device);
			}
		} else{
			continue;
		}
		if (receive_buffer.less_than_min == 0) {
			nbytes = nbytes - ETH_HLEN - 1;
		}

		remote_data_t * rd = (remote_data_t * ) receive_buffer.data;
		char * payload = (char *) (&receive_buffer.data[0] + sizeof(remote_data_t));
		sd.data_src = 32;
		time_us = rd->time;
		sd.len = rd->size;
		sd.is_wmp = rd->type;
		if (sd.is_wmp){

			memcpy(data, payload, rd->size);
			wmpFrame * p = (wmpFrame*) data;
			sd.frame_type = SP_LUS_WMP_FRAME;
			poses[p->hdr.from].reached=true;
		}else{
			sd.frame_type = SP_FOREIGN;
		}
		return rd->size;
	}
	close(sock);
	fprintf(stdout,"OK!\n");
	return 0;
}

