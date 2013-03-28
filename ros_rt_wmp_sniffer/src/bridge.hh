/*------------------------------------------------------------------------
 *---------------------           WMPSNIFFER          --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: bridge.hh
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
#ifndef BRIDGE_HH_
#define BRIDGE_HH_

#include "buffer_layer.h"

#define SIM_LINUX_US        1
#define REAL_LINUX_US       2
#define REAL_MARTE          3
#define REAL_LINUX_KS       4
#define REAL_LINUX_PCAP     5

int start_bridge(int num_nodes_p, int _sim);
void stop_bridge();
void start_batch();
void io_init();
int io_open_sim_data(char * filename);
int io_get_num_nodes();
int io_get_file_size();
int io_go_to(int n);
int io_read_next_sim_data(char * p);
int io_read_sim_data(char * p, int pos);
int io_get_sim_data_num_of_elements();
int io_get_unit_size();
int io_close_sim_data();
bool show_foreign_bridge();

int read_next_token(char * tmp,char* bc_msg,int *serial,int * pos,const char * txt);
int read_next_message(char * tmp,char* bc_msg,int *serial,int * pos,const char * txt);
int read_next_drop(char * tmp,char* bc_msg,int *serial,int * pos,const char * txt);
int read_next_authorization(char * tmp,char* bc_msg,int *serial,int * pos,const char * txt);
int read_next_bc(char * tmp,char * bc_msg,int *serial,int * pos,const char * txt);

int get_actual_gct(int *best,int *act);
void reset_actual_gct();
#include <string>
std::string operator*(const std::string & s, int n);


#endif /*BRIDGE_HH_*/


