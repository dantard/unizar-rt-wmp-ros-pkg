/*------------------------------------------------------------------------
 *---------------------           WMPSNIFFER          --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: basic_io.h
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
#ifndef FILE_IO_H_
#define FILE_IO_H_

#include <cstdio>
#include "wmp_specific.hh"

#define MAX_FRAME_SIZE 2000 //TODO: WAS 200

struct file_t {
	FILE * ptr;
	bool open;
	char name[256];
	int pos;
};

typedef struct  {
	unsigned long long time;
	unsigned char num_nodes;
	int frame_type;
	int len;
	double rate;
	bool is_wmp;
	int time_source;
	char data_src;
	unsigned int key;
	long long t1;
	long long t2;
	bool t1t2_valid;
	unsigned long long onair_local_ts;
	bool onair_local_ts_valid;
	int simDataLen;
	int proto;
	int reinserted;
} simData_Hdr;

//typedef struct {
//	double x,y,a;
//	unsigned char reached;
//	unsigned char extra;
//	bool pose_is_valid;
//} simData_Node;





void io_init() ;
int io_open_sim_data(char * filename) ;
int io_close_sim_data() ;
int io_get_num_of_nodes() ;
int io_get_file_size() ;
int io_go_to(int n) ;
int io_read_next_sim_data(char * p);
int io_read_sim_data(char * p, int pos) ;
int io_get_sim_data_num_of_elements() ;
int io_reopen_file_to_write(int n_nodes) ;
int io_get_filepos();
int io_write_sim_frame(char * fdata, int data_size);
int io_get_pose_from_serial(int serial);
int io_change_file();
int io_change_file(char *p);

void io_flush();

int io_write_on_tmp_file(char * p, int size);
int io_create_tmp_file();
int io_close_tmp_file();

#endif /* FILE_IO_H_ */
