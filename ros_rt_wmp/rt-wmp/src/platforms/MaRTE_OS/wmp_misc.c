/*------------------------------------------------------------------------
 *---------------------           RT-WMP              --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: ./src/platforms/MaRTE_OS/wmp_misc.c
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

#include "config/compiler.h"
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#include <stdio.h>
#include "core/include/wmp_misc.h"


static unsigned long long timeZero=0;
static int initialized=0;

unsigned long long getActualTimems(){
	struct timeval tv;
	gettimeofday(&tv,NULL);
	unsigned long long tmp=(tv.tv_sec)*1000000LL;
	tmp+=tv.tv_usec;
	tmp=tmp-timeZero;
	tmp=(unsigned long long) ((double)tmp/1000.0);
	return tmp;
}

unsigned long long getActualTimeus(){
	if (!initialized) {
		initialized=1;
		struct timeval tv;
		gettimeofday(&tv,0);
		unsigned long long tmp=(tv.tv_sec)*1000000LL;
		tmp+=tv.tv_usec;
		timeZero=tmp;
	}
	struct timeval tv;
	gettimeofday(&tv,0);
	unsigned long long tmp=(tv.tv_sec)*1000000LL;
	tmp+=tv.tv_usec;
	tmp=tmp-timeZero;
	return tmp;

}

void mssleep(int ms){
	struct timespec ts;

	ts.tv_sec = 0;
    ts.tv_nsec = ms*1000*1000;
    nanosleep (&ts, NULL);
}

void ussleep(int us){
	struct timespec ts;

	ts.tv_sec = 0;
    ts.tv_nsec = us*1000;
    nanosleep (&ts, NULL);
}

unsigned long long getActualDateus(){
	unsigned long long timer=0;
	struct timeval tv;
	gettimeofday(&tv,0);
	unsigned long long tmp=(tv.tv_sec)*1000000LL;
	tmp+=tv.tv_usec;
	return tmp;
}

unsigned long long getRawActualTimeus() {
    struct timeval tv;
    gettimeofday(&tv, 0);
    unsigned long long tim1=tv.tv_sec;
    tim1*=1000000;
    tim1+=tv.tv_usec;
    return tim1;
}

typedef void (*sighandler_t)(int);
	sighandler_t signal(int signum, sighandler_t handler){
	WMP_MSG(stderr,"*** WARNING *** signal not yet implemented in MaRTE_OS\n");
}

#include <time.h>
void wmp_get_timestamp(char *timecad)
{
	char buffer[30];
	struct timeval tv;
	time_t curtime;
	struct tm *t;
	gettimeofday(&tv, NULL);
	curtime = tv.tv_sec;
	t = localtime(&curtime);
	sprintf(timecad, "%02d%02d", t->tm_hour, t->tm_min);
}

int system(char * txt){
	fprintf(stderr,"*** Function 'system' not implemented in MaRTE_OS\n*** Unable to execute %s\n",txt);
	return 0;
}

