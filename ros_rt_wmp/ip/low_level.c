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

#define WMP_MSG fprintf

static int initialized[64];
static int txsockfd, rxsockfd[64];
static struct sockaddr_in txservaddr, rxservaddr[64], cliaddr;
static char bufrx[65536];
static char mactive_nodes;
static char base_ip[20], mnode_id;
static int idx = 0;

static int readllcfg() {
	char filename[256], line[256];
	char param[20], val[20];

    snprintf(base_ip, 20, "%s", "192.168.1.1");
	snprintf(filename, 256, "%s/.rt-wmp/rt-wmp-us-ip.ll", getenv("HOME"));

	FILE * f = fopen(filename, "r");
	if (f > 0) {
		WMP_MSG(stderr, "Reading Low Level Configuration file (%s)... \n",
				filename);
		while (fgets(line, 256, f) != NULL) {

			if (line[0] < 65 || line[0] > 90) {
				continue;
			}
			sscanf(line, "%s %s", param, val);

			int exists = 0;
			if (strcmp(param, "BASE_IP") == 0) {
				strcpy(base_ip, val);
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

	struct in_addr addr;
	addr.s_addr = htonl(ntohl(inet_addr(base_ip)) + 1);
	char *dot = inet_ntoa(addr);
	fprintf(stderr, "Node one has IP: %s\n", base_ip);
	fprintf(stderr, "Node two has IP: %s, etc.\n", dot);

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

int wmpSetup(char node_id, char active_nodes) {
	mnode_id = node_id;
	mactive_nodes = active_nodes;
	readllcfg();

	/* TX */
	txsockfd = socket(AF_INET, SOCK_DGRAM, 0);
	bzero(&txservaddr, sizeof(txservaddr));
	txservaddr.sin_family = AF_INET;
	memset(initialized, 0, sizeof(initialized));
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


int wmpPushData(unsigned int port, char * p, unsigned int size,
		unsigned int dest, signed char priority) {

	int ip = 0;
	while (dest >>= 1) {
		++ip;
	}
	int address = htonl(ntohl(inet_addr(base_ip)) + ip);

	txservaddr.sin_addr.s_addr = address; //inet_addr(base_ip) + ip;
	txservaddr.sin_port = htons(32000 + port);

	sendto(txsockfd, p, size, 0, (struct sockaddr *) &txservaddr,
			sizeof(txservaddr));
    //fprintf(stderr, "Sent to port: %d (dest: %d, ip+:%d)\n", 32000 + port, dest, ip);
	return 1;
}

int wmpPopData(unsigned int port, char ** p, unsigned int * size,
		unsigned char * src, signed char * priority) {
	if (!initialized[port]) {
		fprintf(stderr, "Listening at port: %d\n", 32000 + port);
		rxsockfd[port] = socket(AF_INET, SOCK_DGRAM, 0);
		bzero(&rxservaddr[port], sizeof(rxservaddr));
		rxservaddr[port].sin_family = AF_INET;

        int address = htonl(ntohl(inet_addr(base_ip)) + mnode_id);

        rxservaddr[port].sin_addr.s_addr = address;// htonl(INADDR_ANY);
		rxservaddr[port].sin_port = htons(32000 + port);
        int res = bind(rxsockfd[port], (struct sockaddr *) &rxservaddr[port],
				sizeof(rxservaddr[port]));

        if (res!=0){
            fprintf(stderr,"*** ABORTING *** Bind error: do IP address and node-id are coherent in this machine?\n");
            exit(0);
        }else{
            initialized[port] = 1;
        }
	}

	int len = sizeof(cliaddr);
	*size = recvfrom(rxsockfd[port], bufrx, 65535, 0,
			(struct sockaddr *) &cliaddr, &len);
    //fprintf(stderr, "Received port: %d size:%d\n", 32000 + port, *size);
	*p = bufrx;
	*src = 2;
	*priority = 0;
	return (*size) > 0 ? 1 : -1;
}

int wmpPopDataTimeout(unsigned int port, char ** p, unsigned int * size, unsigned char * src, signed char * priority, int to) {
	return 1;
}

void wmpPopDataDone(int id) {
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

