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

char wmpGetNodeId(void) {
	return 1;
}

char wmpGetNumOfNodes(void) {
	return 1;
}

int wmpGetLatestLQM(char * lqm) {
	return 1;
}

int wmpIsNetworkConnected(void) {
	return 1;
}

int wmpSetup(char node_id, char active_nodes) {
	return 1;
}
void wmpRunBG(void) {
	return;
}

unsigned int wmpGetSerial(void) {
	return 1;
}
unsigned int wmpGetLoopId(void) {
	return 1;
}

int wmpPushData(unsigned int port, char * p, unsigned int size,
		unsigned int dest, signed char priority) {
	return 1;
}
int wmpPopData(unsigned int port, char ** p, unsigned int * size,
		unsigned char * src, signed char * priority) {
	return 1;
}

int wmpPopDataTimeout(unsigned int port, char ** p, unsigned int * size,
		unsigned char * src, signed char * priority, int to) {
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

