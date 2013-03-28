/*------------------------------------------------------------------------
 *---------------------           WMPSNIFFER          --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: buffer_layer.h
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

#ifndef BUFFER_LAYER_H_
#define BUFFER_LAYER_H_

#include <set>
#include <map>
#include "basic_io.h"

#define QUEUE_LEN 			250
#define SYNC_SAMPLE_LEN		250

struct ts_key_t{
	unsigned int key;
	unsigned long long ts;
	int elem;
};

struct base_time_t{
	int count;
	long long time_us;
};

struct frame_t{
	bool rxd;
	unsigned long long time_us;
};

struct  robo_pose_t {
	bool reached;
	bool pose_is_valid;
	double x,y,a;
};

struct  remote_data_t {
	int size;
	int source;
	unsigned long long time;
	int pose_is_valid;
	double x,y,a;
	int type;
};



struct pac {
	simData_Hdr sdh;
	unsigned long long time_us;
	char data[2500];
	int size,data_source;
	std::set<int> reached;
	std::map<int,robo_pose_t> poses;
};

struct sniff_func_t {
	int (*sniff_func)(char * data, simData_Hdr & sd,
			unsigned long long &time_us, std::map<int, robo_pose_t> & poses);
	bool predominant;
	int id;
};

enum sniff_type_t {
	ST_PCAP, ST_SOCKET, ST_MARTE, ST_FILE, ST_SIM, ST_SHARED_MEM
};

enum sniff_pkt_type_t {
	SP_MARTE_WMP_FRAME,SP_LKS_WMP_FRAME,SP_LUS_WMP_FRAME, SP_FOREIGN, SP_LUS_WMP_FRAME_DUP
};

int buffer_layer_init(int _st, int nnodes, char * iface);
int buffer_layer_sniff_packet(char * data, simData_Hdr & sdh,
		unsigned long long & time_us, std::set<int> & reached, std::map<int,robo_pose_t> & poses);
void buffer_layer_stop();
void buffer_layer_clear();
#endif /* BUFFER_LAYER_H_ */
