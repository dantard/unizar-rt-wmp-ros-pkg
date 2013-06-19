/*------------------------------------------------------------------------
 *---------------------           WMPSNIFFER          --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: buff_layer.cc
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
#include <iostream>
#include <semaphore.h>
#include <map>
#include <queue>
#include <set>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include "pcap_layer.h"
#include "wmp_config.h"
#include "core/include/frames.h"
#include "bridge.hh"
#include "basic_io.h"
#include "buffer_layer.h"
#include "marte_layer.h"
#include <cassert>
#include <cstdlib>
#include <map>
#include <algorithm>
#include <assert.h>
#include "misc.h"
#include "BoundedHash.h"
#include "shmem_layer.h"
extern "C" {
 #include "core/include/wmp_utils.h"
}

#define EXIT_KEY 1

static std::map<int, pac> rxMap;
static std::vector<ts_key_t> keyQueue;
static BoundedHash<int, pac, unsigned long long> bh;
static std::map<int, pac> rxMap2;
static std::vector<int> keyQueue2;
static sem_t rxsem;
static pthread_mutex_t mex;
static bool bl_keep_running = false;

static bool synchronized;
static int num_nodes;
static int st;// = (int*) v;
static int foreign_idx = 100;

static base_time_t bt[33];





int wmpGetFrameHash(wmpFrame * p){
	return (p->hdr.serial * 10000 + p->hdr.from * 1000 + p->hdr.to
			* 100 + p->hdr.retries * 10 + p->hdr.type);//+sdh.frame_type;
}
bool active = false;
void buffer_layer_activate(bool activate){
	active = activate;

}

int lastPopped;
void buffer_layer_clear(){
	rxMap.clear();
	rxMap2.clear();
	keyQueue2.clear();
	keyQueue.clear();
	bh.clear();
}

int buffer_layer_sniff_packet(char * data, simData_Hdr & sdh,
		unsigned long long & time_us, std::set<int> & reached, std::map<int,
				robo_pose_t> & poses) {
	pac pk;
	wmpFrame * f = (wmpFrame *) data;
	wmpFrame lastFrame;
	lastFrame.hdr.serial = 0;   
	unsigned long long lastTime;
	while (1) {
		pk = bh.pop();
		if (pk.size == 0){
			return 0;
		}
		if (pk.data_source != 32) {
			bool cond = lastFrame.hdr.serial + 1 == f->hdr.serial;
			if (!cond) {
				if (bh.countk2(f->hdr.serial) == 1) {
					int key = bh.getKey(f->hdr.serial, 0);
					pk = bh.pop(key);
					pk.sdh.time_source = 99;
				} else {
					continue;
				}
			}
		}
		memcpy(data, pk.data, pk.size);

		sdh = pk.sdh;
		time_us = pk.time_us;
		if (pk.sdh.frame_type != SP_FOREIGN || lastPopped != SP_FOREIGN) {
			break;
		}
	}
	lastTime = pk.time_us;
	lastFrame = *f;
	lastPopped = pk.sdh.frame_type;
	reached = pk.reached;
	poses = pk.poses;
	return pk.size;
}

void* loop(void * v) {
	char buf[2500];
	simData_Hdr sdh;
	unsigned long long time_us;
	wmpFrame * p = (wmpFrame *) buf;
	sniff_func_t * sf = (sniff_func_t *) v;
	static int ok=0,fore=0;

	while (bl_keep_running) {

		std::map<int, robo_pose_t> poses;
		poses.clear();

		/* SNIFFING */
		int size = sf->sniff_func(buf, sdh, time_us, poses);

		if (!active){
			buffer_layer_clear();
			continue;
		}

		unsigned int key;
		if (size > 0) {
			bh.sleep();
			bh.lock();
			if (sdh.frame_type != SP_FOREIGN) {
				key = wmpGetFrameHash(p);
				ok++;
			} else {
				fore++;
				key = foreign_idx++;
				if (!show_foreign_bridge()){
					bh.unlock();
					continue;
				}
			}

			fprintf(stderr,"Frames: %d, Foreign: %d - Serial: %d \r",ok,fore, (int) p->hdr.serial);


			bool present = bh.find(key);
			if (not present) {
				pac pkt;
				pkt.size = size;
				pkt.sdh.data_src = sdh.data_src;
				pkt.sdh.key = sdh.key;
				pkt.sdh.is_wmp = sdh.is_wmp;
				pkt.sdh.frame_type = sdh.frame_type;
				pkt.sdh.len = sdh.len;
				pkt.data_source = 99;
				pkt.time_us = time_us;//getRawActualTimeus();
				pkt.sdh.reinserted = sdh.reinserted;
				memcpy(pkt.data, buf, size);

				//bh.lock();
				bh.insert(key, p->hdr.serial, pkt.time_us) = pkt;
				//bh.unlock();
			}
			pac & pkt = bh.get(key);
			if (sdh.data_src == 32) {
				pkt.time_us = time_us;
				pkt.sdh.time = time_us;
				pkt.sdh.t1 = time_us;
				pkt.sdh.rate = sdh.rate;
				pkt.sdh.proto = sdh.proto;
				pkt.data_source = 32;
				if (sdh.frame_type == SP_LUS_WMP_FRAME_DUP) {
					if (pkt.sdh.t1 != 0) {
						/* OJO, la SP_LUS_WMP_FRAME_DUP llega DOS veces pero la segunda es la buena*/
						pkt.sdh.t2 = time_us + bt[sdh.data_src].time_us;
						pkt.sdh.t1t2_valid = true;
					}
				}
			} else {
				if (sdh.data_src == p->hdr.from) {
					pkt.sdh.onair_local_ts = sdh.onair_local_ts;
					pkt.sdh.onair_local_ts_valid = sdh.onair_local_ts_valid;
				}
			}
			/* UPDATE POSE < */
			std::map<int, robo_pose_t>::iterator iter;
			for (iter = poses.begin(); iter != poses.end(); ++iter) {
				if (iter->second.pose_is_valid) {
					pkt.poses[iter->first] = iter->second;
				}
				pkt.reached.insert(iter->first);
			}
			bh.sort();
			bh.unlock();
		} else {
			//fprintf(stderr,"PUSHED EXIT_KEY...\n\n");
			pac pkt;
			pkt.size = 0;
			pkt.data_source = 64;
			pkt.sdh.key = 0;
			pkt.time_us = 0;
			bh.lock();
			bh.insert(0, 0, time_us + 1) = pkt;
			bh.drain();
			bh.unlock();
			bl_keep_running=false;
		}
	}
	return 0;
}

sniff_func_t sf;
sniff_func_t sf2;

int buffer_layer_init(int _st, int nnodes, char * iface) {
	bh.clear();
	st = _st;
	bl_keep_running = true;
	pthread_t th1;
	sem_init(&rxsem, 1, 0);
	pthread_mutex_init(&mex, 0);
	synchronized = false;
	num_nodes = nnodes;
	bh.setBufferSize(0);

	memset((char*) bt, 0, sizeof(bt));

	switch (st) {
	case ST_PCAP:
		//if (!pcap_init(iface, nnodes)) return 0;
		sf.sniff_func = pcap_sniff_packet;
		sf.predominant = true;
		sf.id = 11;
		pthread_create(&th1, 0, loop, (void *) &sf);
		break;
	case ST_MARTE:
		marte_layer_init(nnodes);
		sf2.sniff_func = marte_sniff_packet;
		sf2.predominant = false;
		sf2.id = 44;
		pthread_create(&th1, 0, loop, (void *) &sf2);
		break;
	case ST_SHARED_MEM:
		shmem_init(nnodes);
		sf2.sniff_func = shmem_sniff_packet;
		sf2.predominant = false;
		sf2.id = 66;
		pthread_create(&th1, 0, loop, (void *) &sf2);
		break;
	default:
		assert(0);
	}
	return 1;
}

void buffer_layer_stop() {
	//keep_runnning = false;
	switch (st) {
	case ST_PCAP:
		pcap_layer_close();
		break;
	case ST_MARTE:
		marte_layer_close();
		break;
	case ST_SHARED_MEM:
		shmem_close();
		break;
	default:
		assert(0);
	}
}
