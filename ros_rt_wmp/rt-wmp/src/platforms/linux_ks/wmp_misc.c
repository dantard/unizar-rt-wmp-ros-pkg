/*------------------------------------------------------------------------
 *---------------------           RT-WMP              --------------------
 *------------------------------------------------------------------------
 *                                                         
 *
 *
 *  File: ./src/platforms/linux_ks/wmp_misc.c
 *  Authors: Rubén Durán
 *           Danilo Tardioli
 *  ----------------------------------------------------------------------
 *  Copyright (C) 2000-2011, Universidad de Zaragoza, SPAIN
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

#include <linux/time.h>
#include <linux/delay.h>

#include "config/compiler.h"

#define gettimeofday(p_tv, foo) do_gettimeofday(p_tv)

unsigned long long timeZero=0;


unsigned long getActualTimems(void) {
   struct timeval tv;
   unsigned long long tmp;
   long timer;

	static int initialized = 0;
	timer = 0;
	if (!initialized) {
		initialized = 1;
		gettimeofday(&tv, 0);
		tmp = (tv.tv_sec) * 1000000LL;
		tmp += tv.tv_usec;
		timeZero = tmp;
	}

	gettimeofday(&tv, 0);
	tmp = (tv.tv_sec) * 1000000LL;
	tmp += tv.tv_usec;
	tmp = tmp - timeZero;
	tmp = DO_DIV64(tmp, 1000);
	return (unsigned long) tmp;
}


unsigned long long getActualTimeus(void){
   struct timeval tv;
   unsigned long long tmp;
   long timer;

	static int initialized = 0;
	timer=0;
	if (!initialized) {
		initialized=1;
		gettimeofday(&tv,0);
		tmp=(tv.tv_sec)*1000000LL;
		tmp+=tv.tv_usec;
		timeZero=tmp;
	}

	gettimeofday(&tv,0);
	tmp=(tv.tv_sec)*1000000LL;
	tmp+=tv.tv_usec;
	tmp=tmp-timeZero;
	return tmp;
}

void mssleep(int ms){
	msleep(ms);
}

unsigned long long getActualDateus(void){
		unsigned long long tmp;
		struct timeval tv;
		gettimeofday(&tv,0);
		tmp=(tv.tv_sec)*1000000LL;
		tmp+=tv.tv_usec;
		return tmp;
}

unsigned long long getRawActualTimeus(void) {
   unsigned long long tim1;
   struct timeval tv;
   gettimeofday(&tv, 0);
   tim1=tv.tv_sec;
   tim1*=1000000;
   tim1+=tv.tv_usec;
   return tim1;
}


void wmp_get_timestamp(char *timecad){
   struct timeval tiempo;
   struct tm strtm;
   do_gettimeofday(&tiempo);
   time_to_tm(tiempo.tv_sec, -60*sys_tz.tz_minuteswest, &strtm);
   snprintf(timecad, 80, "%d-%02d-%02d_%02d-%02d",
            (int)(strtm.tm_year + 1900),   /* Years since 1900 */
            strtm.tm_mon + 1,       /* Range 0 to 11, so we add 1 */
            strtm.tm_mday,          /* Day of the month, in the range 1 to 31 */
            strtm.tm_hour,
            strtm.tm_min);
}

