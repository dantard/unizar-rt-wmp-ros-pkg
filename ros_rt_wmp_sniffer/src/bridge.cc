/*------------------------------------------------------------------------
 *---------------------           WMPSNIFFER          --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: bridge.cc
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
#include "bridge.hh"
#include <sys/types.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include "errno.h"
#include <pthread.h>
#include "area.h"
#include "MyArea.hh"
#include "main_window.hh"
#include <cmath>
#include <cstdio>
#include <sys/io.h>
#include <fcntl.h>
#include <sys/stat.h>
#include "marte_layer.h"
#include "core/interface/Msg.h"
#include "wmp_config.h"

#include "core/include/frames.h"
#include "area_wrapper.h"
#include "pcap_layer.h"
#include "statistics.h"
#include "sstream"
#include "fstream"
#include "buffer_layer.h"
#include <set>
#include <iostream>
#include "wmp_specific.hh"
#include "basic_io.h"
#include "gct.h"

pthread_mutex_t sem;
int sim;
struct sockaddr_in tx_addr, rx_addr;

extern char iface[16];
int  tx, rx, unit_size,filepos = 0, delay, DIST_MAX,
	THRESHOLD = 500, num_nodes, clicked = 0,stopped = 0;

void stop_bridge() {
	stopped = 1;
}

void* bridge(void * param) {
    /* BRIDGE */
    int index = 0;
    char f[2500], fdata[2500];
    unsigned long long time_us,old_time_us = 0,base_time_us=0;
    wmpFrame * p = (wmpFrame*) f;
    bool first_frame = true;

	unsigned long long started_at_us = getRawActualTimeus();

	while (stopped != 1) {

		simData_Hdr sd_hdr;
      	std::set<int> reached;
      	std::map<int, robo_pose_t> poses;
        int nbytes = 0, data_size = 0;
        char * pointer = &fdata[0];

        nbytes=pcap_sniff_packet(f,sd_hdr,time_us,poses);

        if (nbytes < 0){
        	continue;
        }

        if (time_us < started_at_us){
        	continue;
        }

        if (first_frame){
        	base_time_us=time_us;
        	first_frame = false;
        }

        time_us-=base_time_us;
       	if (nbytes <= 0){
        	stopped = 1;
       		continue;
        }

        /* check frame validity */
        if (sd_hdr.is_wmp){
        	if (!valid_frame(p, nbytes,num_nodes)) continue;
        	greatest_clean_time_check(p);
        }

        sd_hdr.time=time_us;
        sd_hdr.num_nodes = num_nodes;
        sd_hdr.len = nbytes;

        old_time_us = time_us;
        index++;

        simData_Hdr * sdhp = (simData_Hdr *) &fdata[0];

        memcpy(pointer, (char*) & sd_hdr, sizeof (sd_hdr));
        pointer += sizeof (sd_hdr);
        data_size += sizeof (sd_hdr);

		int flen = wmp_get_frame_total_lenght(p,num_nodes);
		flen = nbytes;
		memcpy(pointer, (char*) f, flen);


        pointer += flen;
        data_size += flen;
        sdhp->simDataLen = data_size;
        io_write_sim_frame(&fdata[0], data_size);
        new_frame(fdata, data_size);
    }
    close(rx);
    close(tx);
    SIGNAL(&sem);
    io_close_sim_data();
    informBridgeOff();
    return 0;
}

int start_bridge(int num_nodes_p, int _sim) {
	sim = _sim;
    num_nodes = num_nodes_p;
    reset_actual_gct();
    io_reopen_file_to_write(num_nodes);
    pthread_mutex_init(&sem, NULL);
    WAIT(&sem);
    stopped = 0;
    pthread_t tid;
    pthread_create(&tid, NULL, bridge, NULL);
    return 1;
}

bool show_foreign_bridge(){
	return showForeign();
}
