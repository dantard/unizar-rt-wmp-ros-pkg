/*------------------------------------------------------------------------
 *---------------------           RT-WMP              --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: ./src/core/include/wmp_misc.h                                   
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



#ifndef WMP_MISC__H
#define WMP_MISC__H
#include "definitions.h"
//#include "config/compiler.h"

unsigned long long getActualDateus(void);
unsigned long long getActualTimeus(void);
unsigned long long getActualTimems(void);
unsigned long long getRawActualTimeus(void);

void mssleep(int ms);
void ussleep(int us);

void state_to_string(char state, char* string);
void wmpSetDefaultConfiguration(Status * s);

unsigned char get_net_id(void);
void wmp_get_timestamp(char *timecad);

void wmp_set_pose(double x, double y, int pow_mw);
void wmp_get_pose(double * x, double * y, int * pow);
int apply_config(Status * s, char * param, char * val);

#endif

