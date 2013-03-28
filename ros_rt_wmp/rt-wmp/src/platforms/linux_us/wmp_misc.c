/*------------------------------------------------------------------------
 *---------------------           RT-WMP              --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: ./src/platforms/linux_us/wmp_misc.c                             
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

#include <sys/time.h>
#include <unistd.h>
#include <stdio.h>
#include <time.h>

unsigned long long timeZero=0;


unsigned long getActualTimems() {
	static int initialized = 0;
	long timer = 0;
	if (!initialized) {
		initialized = 1;
		struct timeval tv;
		gettimeofday(&tv, 0);
		unsigned long long tmp = (tv.tv_sec) * 1000000LL;
		tmp += tv.tv_usec;
		timeZero = tmp;
	}

	struct timeval tv;
	gettimeofday(&tv, 0);
	unsigned long long tmp = (tv.tv_sec) * 1000000LL;
	tmp += tv.tv_usec;
	tmp = tmp - timeZero;
	tmp = (unsigned long long) ((double) tmp / 1000.0);
	return (unsigned long) tmp;
}


unsigned long long getActualTimeus(){
	static int initialized = 0;
	long timer=0;
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
	int i;
	long us=ms*1000;
	int n_iterations=us /1000000;
	int remaining=us % 1000000;
	for (i=0;i<n_iterations;i++){
		usleep(999999);
	}
	usleep((unsigned int)(remaining*0.99));
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


void wmp_get_timestamp(char *timecad){
	struct tm *tmPtr;
	time_t tiempo;
	tiempo = time(NULL);
	tmPtr = localtime(&tiempo);
	strftime(timecad, 80, "%Y-%m-%d_%H-%M", tmPtr);
}

void textcolor(int attr, int fg, int bg)
{	char command[13];
	/* Command is the control command to the terminal */
	sprintf(command, "%c[%d;%d;%dm", 0x1B, attr, fg + 30, bg + 40);
	fprintf(stderr,"%s", command);
}
