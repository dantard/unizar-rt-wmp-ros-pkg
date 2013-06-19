/*------------------------------------------------------------------------
 *---------------------           RT-WMP              --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: ./src/core/include/wmp_utils.h                                  
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



#ifndef WMP_UTILS_H_
#define WMP_UTILS_H_

#include "../include/frames.h"

long long wmp_timespec_to_ns(struct timespec *ts);
long long wmp_elapsed_ms (struct timespec *begin, struct timespec *end);
void wmp_add_ms(struct timespec *ts, int ms);
unsigned int wmp_calculate_frame_duration_ms(int rate, int size) ;
unsigned int wmp_calculate_frame_duration_us(int rate, int size) ;

int wmpGetFrameHash(wmpFrame * p);
int wmpGetFrameHashNoRetry(wmpFrame * p);

void lqmToString(char **lqm, char * s, int nnodes);
void wmp_print_lqm(char ** lqm, char * s, int nnodes);
void wmp_print(const char *format, ...);
int wmp_print_get_size(void);
void wmp_print_reset(void);
int wmp_print_put(wmpFrame * p);
int ms_to_us(int n);
int us_to_ms(int n);
#endif /* WMP_UTILS_H_ */

