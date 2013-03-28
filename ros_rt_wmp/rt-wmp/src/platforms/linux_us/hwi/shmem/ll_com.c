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

#include <unistd.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <stdio.h>
#include <semaphore.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
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

#define SHMSZ 2500
static char *shm, *s;
static sem_t *mutex[32], *sniff_mutex, *sniff_mutex2, *dest_mutex;
static int shmid, sniffer_present;

int readllcfg() {
	return 0;
}

void closeLowLevelCom() {
	int i;

	sem_unlink("wmp_sem_sniff");
	sem_close(sniff_mutex);

	for (i = 0; i < wmpGetNumOfNodes(); i++) {
		char SEM_NAME[32];
		sprintf(SEM_NAME, "wmp_sem_%d", i);
		sem_unlink(SEM_NAME);
		sem_close(mutex[i]);
	}
	shmctl(shmid, IPC_RMID, 0);
}

int initLowLevelCom() {
	int i;
	char SEM_NAME[32];
	key_t key = 0x6969, key2=0x1111;
	//closeLowLevelCom();

	fprintf(stderr,"Checking for sniffer...\n");
	char path[256];
	sprintf(path,"%s/.wmpSniffer/active",getenv("HOME"));
	FILE * val = fopen(path,"r");
	if (val>0) {
		sniffer_present = 1;
		fclose(val);
		fprintf(stderr,"*** WARNING: Sniffer is present, if it is not, delete %s\n",path);
	} else{
		sniffer_present = 0;
	}

	sniff_mutex = sem_open("wmp_sem_sniff", O_CREAT, 0644, 0);
	sniff_mutex2 = sem_open("wmp_sem_sniff2", O_CREAT, 0644, 0);
	dest_mutex = sem_open("dest_sem", O_CREAT, 0644, 0);

	if (sniff_mutex == SEM_FAILED || sniff_mutex2 == SEM_FAILED) {
		perror("Unable to create sniffer semaphores");
		sem_unlink("wmp_sem_sniff");
		sem_unlink("wmp_sem_sniff2");
		exit(-1);
	}
	for (i = 0; i < wmpGetNumOfNodes(); i++) {
		sprintf(SEM_NAME, "wmp_sem_%d", i);
		mutex[i] = sem_open(SEM_NAME, O_CREAT, 0644, 0);
		if (mutex[i] == SEM_FAILED) {
			perror("Unable to create semaphore");
			sem_unlink(SEM_NAME);
			exit(-1);
		}
	}

	/* shared memory creation */
	shmid = shmget(key, SHMSZ, IPC_CREAT | 0666);
	if (shmid < 0) {
		perror("Failure in shmget");
		exit(-1);
	}
	shm = shmat(shmid, NULL, 0);

	int n=0;
	while (!sem_trywait(mutex[wmpGetNodeId()])){
		n++;
	}
	while (!sem_trywait(sniff_mutex)){
	}
	while (!sem_trywait(sniff_mutex2)){
	}
	while (!sem_trywait(dest_mutex)){
	}


	WMP_MSG(stderr,"Discharging semaphore (%d)\n",n);
}
#include "../../../../core/include/wmp_utils.h"
int llsend(char * f, int size) {
	int i, *p;
	p = (int *) shm;
	(*p) = size;

	// 1Mbps
	double dsize = size, dur = 292.0 + (28.0 + size)/1e6;
	usleep(dur);

	memcpy(shm + sizeof(int), f, size);

	if (sniffer_present){
		sem_post(sniff_mutex);
		sem_wait(sniff_mutex2);
	}


	for (i = 0; i < wmpGetNumOfNodes(); i++) {
		if (i != wmpGetNodeId()) {
			sem_post(mutex[i]);
		}
	}

	struct timespec ts;
	clock_gettime(CLOCK_REALTIME,&ts);
	wmp_add_ms(&ts,wmpGetNumOfNodes());
//	sem_timedwait(dest_mutex,&ts);

	return size;
}

rxInfo llreceive(char *f, int timeout) {
	int ret;
	rxInfo rxi;
	struct timespec ts;

	clock_gettime(CLOCK_REALTIME, &ts);
	wmp_add_ms(&ts, timeout);

	ret = sem_timedwait(mutex[wmpGetNodeId()], &ts);

	wmpFrame * fr = (wmpFrame*) shm + sizeof(int);

	if (fr->hdr.to == wmpGetNodeId()){
		sem_post(dest_mutex);
	}

	if (ret == 0) {
		int *len = (int *) shm;
		memcpy(f, shm + sizeof(int), (*len));
		rxi.size = (*len);
		rxi.error = 0;
		rxi.proto = 0x6969;
	} else {
		rxi.error = 1;
	}
	return rxi;
}

void llsetPower(int dbm){

}
