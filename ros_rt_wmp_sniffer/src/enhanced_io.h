/*------------------------------------------------------------------------
 *---------------------           WMPSNIFFER          --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: enhanced_io.h
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
#ifndef ENHANCED_IO_H_
#define ENHANCED_IO_H_

#include "basic_io.h"
/*
 * enhanced_io.cc
 *
 *  Created on: May 28, 2009
 *      Author: danilo
 */

int read_next_token(char * msg, char* bc_msg, int *serial, int * pos, const char * txt);
int read_next_message(char * tmp, char * bc_msg, int *serial, int * pos, const char * txt);
int read_next_bc(char * tmp, char * bc_msg, int *serial, int * pos, const char * txt);
int read_next_drop(char * tmp, char * bc_msg, int *serial, int * pos, const char * txt);
int read_next_authorization(char * tmp, char * bc_msg, int *serial, int * pos, const char * txt);

void logger_init(int type, int nnodes_);
void logger_printData(int node_id,wmpFrame *q, unsigned long long ts);
void logger_close();
#endif /* ENHANCED_IO_H_ */
