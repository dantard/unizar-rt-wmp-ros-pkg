/*------------------------------------------------------------------------
 *---------------------           RT-WMP              --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: ./src/core/include/nstat.h                                      
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



#ifndef NSTAT_H_
#define NSTAT_H_

#include "config/compiler.h"

void init_nstat(int n);
void free_nstat(void);
char nstat_get_byte(int idx);
void nstat_set_byte(int idx,char val);

void nstat_set_val(int idx, unsigned char id);
unsigned char nstat_get_val(int idx);

int nstat_isReached(int idx);
void nstat_clearReached(int idx);
void nstat_setReached(int idx);

int nstat_isLost(int idx);
void nstat_setLost(int idx);
void nstat_clearLost(int idx);

int nstat_hasFailed(int idx);
void nstat_clearFailed(int idx);
void nstat_setFailed(int idx);

#endif /*NSTAT_H_*/

