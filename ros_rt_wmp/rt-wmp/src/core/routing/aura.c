/*
 * aura.c
 *
 *  Created on: Sep 22, 2010
 *      Author: danilo
 */

#include "config/compiler.h"
#include "core/include/aura.h"
#include "core/include/frames.h"
#include "core/include/global.h"
#include "core/include/dijkstra.h"
#include "core/include/lqm.h"

static int aura_vec[32];
static char buff[2500];
static int len;
static Message bm;

void aura_store_msg(Message * m){
	char * dp = (char *) m;
	dp += sizeof(Message);

	//XXX: WARNING with tree routing, it should be changed the data_pointer
	//dp += sizeof(Message) + status.N_NODES;
	bm = *m;
	len = m->len;
	memcpy(buff, dp,len);
}

void aura_restore_msg(Message * m){
	char * dp = (char *) m;
	dp += sizeof(Message);

	//XXX: WARNING with tree routing, it should be changed the data_pointer
	//dp += sizeof(Message) + status.N_NODES;
	m->part_id = bm.part_id;
	m->len = bm.len;
	m->msg_hash = bm.msg_hash;
	m->port = bm.port;
	m->priority = bm.priority;
	m->age = bm.age;
	memcpy(dp, buff, len);
	m->len = len;
}

void aura_add(aura_t val, int id){
	aura_vec[id] += val;
}

void aura_clear(void){
	int i;
	for (i = 0; i < 32 ; i++){
		aura_vec[i] = 0;
	}
}

int aura_get(int id){
	return aura_vec[id];
}

void aura_discard_unnecessary(int dest){
	char path[32], necessary[32];
	int i,j;

	memset(necessary,0,sizeof(necessary));
	necessary[status.id] = 1;
	//fprintf(stderr,"### dest: %d\n",dest);
	for (i = 0; i< status.N_NODES ;i++){
		if (mBitsIsSet(dest,i)){
			int len = getPath(lqm_get_ptr(),status.id,i,path);
			//fprintf(stderr,"### path from: %d to %d: ",status.id,i);
			for (j = 0; j < len+1; j++){
				//fprintf(stderr,"%d ",path[j]);
				int node = path[j];
				necessary[node] = 1;
			}
			//fprintf(stderr,"\n");
		}
	}
	for (i = 0; i< status.N_NODES ;i++){
		if (necessary[i] == 0){
			aura_vec[i] = aura_full;
			//fprintf(stderr,"### discard: %d\n",i);
		}
	}
}


int aura_get_next(wmpFrame * p, aura_t * type){
	char nb[32];
	int n_nb = getNeighbors(lqm_get_ptr(),status.id,nb);
	int i, best_val = 1000, best_id = -1;
	for (i = 0; i< n_nb ;i++){
		int elem = aura_vec[(int)nb[i]];
		if (elem < best_val){
			best_val = elem;
			best_id = nb[i];
		}
	}
	if (best_val == 0){
		*type = aura_msg;
		return best_id;
	} else if (best_val == 1){
		*type = aura_msg;
		return best_id;
	} else if (best_val == 2){
		*type = aura_auth;
		return best_id;
	}else{
                //WMP_ERROR(stderr,"Value:%d serial:%u\n",best_val,(int) p->hdr.serial);
		return -1;
	}
}
