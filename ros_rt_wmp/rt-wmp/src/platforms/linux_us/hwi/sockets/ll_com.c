/*------------------------------------------------------------------------
 *---------------------           RT-WMP              --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: ./src/platforms/linux_us/hwi/sockets/ll_com.c
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

#include <sys/types.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "errno.h"
#include <pthread.h>
#include "core/include/definitions.h"
#include "core/interface/wmp_interface.h"
#include "core/include/wmp_misc.h"
#include "core/include/frames.h"
#include "config/compiler.h"
#include "core/include/ll_com.h"

static int rx, tx; /* TX and RX sockets*/
static char IPBROADCAST[20];
static char param[20], val[20];

struct sockaddr_in rx_addr, tx_addr;

int readllcfg() {
	char filename[256], line[256];

	sprintf(IPBROADCAST, "192.168.1.255");

	snprintf(filename, 256, "%s/.rt-wmp/rt-wmp-us-sock.ll", getenv("HOME"));
	FILE * f = fopen(filename, "r");
	if (f > 0) {
		WMP_MSG(stderr,"Reading Low Level Configuration file (%s)... \n",filename);
		while (fgets(line,256,f) != NULL) {
			if (line[0]<65 || line[0]>90){
				continue;
			}
			sscanf(line,"%s %s",param,val);
			int exists = 0;
			if (strcmp(param, "IPBROADCAST") == 0) {
				strcpy(IPBROADCAST, val);
				exists = 1;
			}

			if (exists) {
				WMP_MSG(stderr, "READ OPTION: %s = %s\n", param, val);
			} else {
				WMP_MSG(stderr, "*** UKNOWN OPTION %s = %s\n", param, val);
			}
			WMP_MSG(stderr,"Done.\n");
		}
	} else {
		WMP_MSG(stderr,"File %s not found, using default values\n",filename);
	}
	return 0;
}

void closeLowLevelCom() {
	close(rx);
	close(tx);
}

int initLowLevelCom() {
	int ret = 1, txport = 0x6969, rxport = 0x6969;
	readllcfg();

	WMP_MSG(stderr,"Using IP: %s\n",IPBROADCAST);

	memset(&rx_addr, 0, sizeof(rx_addr));
	rx_addr.sin_family = PF_INET;
	rx_addr.sin_port = htons(rxport);
	rx_addr.sin_addr.s_addr = inet_addr(IPBROADCAST);

	memset(&tx_addr, 0, sizeof(tx_addr));
	tx_addr.sin_family = PF_INET;
	tx_addr.sin_port = htons(txport);
	tx_addr.sin_addr.s_addr = inet_addr(IPBROADCAST);

	if ((rx = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
		WMP_MSG(stderr,"Error creating RX socket.\n");
		ret = 0;
	}

	if (bind(rx, (struct sockaddr*) &rx_addr, sizeof(struct sockaddr_in)) < 0) {
		WMP_MSG(stderr,"RX Bind Error: % s.\n",inet_ntoa(rx_addr.sin_addr));
		ret = 0;
	}

	if ((tx = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
		WMP_MSG(stderr,"Error creating TX socket.\n");
		ret = 0;
	}

	int val = 1, i;
	setsockopt(tx, SOL_SOCKET, SO_BROADCAST, &val, sizeof(val));
	setsockopt(rx, SOL_SOCKET, SO_BROADCAST, &val, sizeof(val));
	setsockopt(rx, SOL_SOCKET, SO_REUSEADDR, &val, sizeof(val));

	if (connect(tx, (struct sockaddr*) &tx_addr, sizeof(struct sockaddr_in))
			< 0) {
		WMP_MSG(stderr,"Connect error: %s.\n",inet_ntoa(tx_addr.sin_addr));
		ret = 0;
	}
	return ret;
}

int llsend(char * f, int size) {
	int nbytes = send(tx, f, size, 0);
	recvfrom(rx, 0, 0, 0, 0, 0);
	return nbytes;
}

rxInfo llreceive(char *f, int timeout) {

	int r = 0;
	struct timeval tv;
	rxInfo ret;

	ret.rate = 0;
	ret.has_lq = 0;

	if (timeout > 0) {
		fd_set fd_rx;
		tv.tv_sec = 0;
		tv.tv_usec = 1000 * timeout;

		FD_ZERO(&fd_rx);
		FD_SET(rx, &fd_rx);

		r = select(FD_SETSIZE, &fd_rx, NULL, NULL, &tv);
	} else {
		r = 1;
	}
	if (r) {
		/* if something is present read the frame and return the frame */
		int rlen = recvfrom(rx, (char *) f, MTU, 0, 0, 0);
		assert(rlen>0);
		ret.size = rlen;
		ret.proto = 0x6969;
		ret.error = 0;
	} else {
		ret.error = 1;
	}
	return ret;
}
