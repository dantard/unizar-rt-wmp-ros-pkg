/*------------------------------------------------------------------------
 *---------------------           WMPSNIFFER          --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: time.cc
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

#include <time.h>
#include <sys/time.h>

static unsigned int initialized = 0, timeZero = 0;

unsigned long getActualTimeus() {
    long timer = 0;
    if (!initialized) {
        initialized = 1;
        struct timeval tv;
        gettimeofday(&tv, 0);
        unsigned long long tmp = (tv.tv_sec)*1000000LL;
        tmp += tv.tv_usec;
        timeZero = tmp;
    }
    struct timeval tv;
    gettimeofday(&tv, 0);
    unsigned long long tmp = (tv.tv_sec)*1000000LL;
    tmp += tv.tv_usec;
    tmp = tmp - timeZero;
    timer = (long) (tmp);
    return timer;
}
#include <stdio.h>
#include <time.h>

#include <linux/sched.h>
unsigned long long getRawActualTimeus() {
    struct timeval tv;
    gettimeofday(&tv, 0);
    unsigned long long tim1=tv.tv_sec;
    tim1*=1000000;
    tim1+=tv.tv_usec;
    return tim1;
}
