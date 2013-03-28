/*------------------------------------------------------------------------
 *---------------------           WMPSNIFFER          --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: shmem_layer.cc
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

#include <stdio.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <stdio.h>
#include <semaphore.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "shmem_layer.h"

#define SHMSZ 2500

static int shmem_keep_running = 1;
static char *shm;
static sem_t * mutex,*mutex2;
static int shmid, shmid2, nnodes, *sniff_present;
static key_t key2 = 0x1111;

int shmem_pre_init(){
	/* sniffer present */
	shmid2 = shmget(key2, sizeof(int), IPC_CREAT | 0666);
	if (shmid2 < 0) {
		perror("failure in shmget");
		exit(-1);
	}
	 sniff_present = (int*) shmat(shmid2, NULL, 0);
	(*sniff_present) = 0x6789;
	return 0;
}

int shmem_init(int _nnodes){

	nnodes = _nnodes;
	key_t key = 0x6969;

	mutex = sem_open("wmp_sem_sniff", O_CREAT, 0644, 0);
	mutex2 = sem_open("wmp_sem_sniff2", O_CREAT, 0644, 0);
	if (mutex == SEM_FAILED) {
		perror("unable to create semaphore");
		sem_unlink("wmp_sem_sniff");
		exit(-1);
	}

	/* shared memory allocation */
	shmid = shmget(key, SHMSZ, IPC_CREAT | 0666);
	if (shmid < 0) {
		perror("failure in shmget");
		exit(-1);
	}
	shm = (char*) shmat(shmid, NULL, 0);


	/* sniffer present */
	shmid2 = shmget(key2, sizeof(int), IPC_CREAT | 0666);
	if (shmid2 < 0) {
		perror("failure in shmget");
		exit(-1);
	}
	 sniff_present = (int*) shmat(shmid2, NULL, 0);
	(*sniff_present) = 0x6789;

	while (!sem_trywait(mutex)){}
	while (!sem_trywait(mutex2)){}

	shmem_keep_running = 1;
	return 1;
}


int shmem_sniff_packet(char * data, simData_Hdr & sd, unsigned long long &time_us, std::map<int,robo_pose_t> & poses) {
	int ret, *len, nbytes;

	(*sniff_present) = 0x6789;
	if (shmem_keep_running){

		ret = sem_wait(mutex);

		len = (int*) shm;
		nbytes = (*len);
		if (nbytes <= 0) {
			return 0;
		}
		memcpy(data, shm + sizeof(int), nbytes);

		time_us=getActualTimeus();
		sd.time=time_us;
		sd.len=nbytes;
		sd.num_nodes=nnodes;
		sd.is_wmp=1;
		sd.data_src = 32;
		sd.frame_type=SP_LUS_WMP_FRAME;
	    wmpFrame * f = (wmpFrame*) data;
		poses[f->hdr.from].reached=true;

		sem_post(mutex2);

		return nbytes;
	}
	return 0;
}

int shmem_close(){
	shmem_keep_running = 0;
	return 0;
}
