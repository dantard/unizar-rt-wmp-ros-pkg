/*------------------------------------------------------------------------
 *---------------------           RT-WMP              --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: ./src/core/wmp_utils.c
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
#include "include/wmp_utils.h"
#include "include/wmp_misc.h"
#include "include/lqm.h"
#include "interface/wmp_interface.h"
long long wmp_timespec_to_ns(struct timespec *ts){
    long long val = (long long) ts->tv_sec * 1000000000 + (long long) ts->tv_nsec;
    return val;
}

long long wmp_elapsed_ms (struct timespec *begin, struct timespec *end){
   long long diff = DO_DIV64(wmp_timespec_to_ns(end), 1000000LL) - DO_DIV64(wmp_timespec_to_ns(begin), 1000000LL);
	return diff;
}

void wmp_add_ms(struct timespec *ts, int ms){
	long long long_ms, sec, nsec;
	long_ms = ((long long)ms * 1000 * 1000) + ts->tv_nsec;
   sec = DO_DIV64(long_ms , (1000 * 1000 * 1000));
   nsec = long_ms - ((long long)sec*(1000 * 1000 * 1000));
	ts->tv_sec  += sec;
	ts->tv_nsec = nsec;
}


unsigned int wmp_calculate_frame_duration_ms(int rate, int size) {
	int res = (242 + (10 * (28 + size * 8) / rate));
	res++;
	return res;
}

unsigned int wmp_calculate_frame_duration_us(int rate, int size) {
	int res = (242 + (10 * (28 + size * 8) / rate));
	return res;
}

int wmpGetFrameHash(wmpFrame * p){
	return (p->hdr.serial * 10000 + p->hdr.from * 1000 + p->hdr.to
			* 100 + p->hdr.retries * 10 + p->hdr.type);//+sdh.frame_type;
}

int wmpGetFrameHashNoRetry(wmpFrame * p){
	return (p->hdr.serial * 10000 + p->hdr.from * 1000 + p->hdr.to * 10 + p->hdr.type);
}

void lqmToString(char **lqm, char * s, int nnodes){
	 int i,j;
	 char txt[10], s1[10];
         int v=0;

	 sprintf(s,"%s","    ");
	 for (i=0;i<nnodes;i++){
	 	sprintf(txt,"%d   ",i);
	 	strcat(s,txt);
	 }

	 strcat(s,"\n");
     for (i=0;i<nnodes;i++){
	 	sprintf(txt,"%d ",i);
        strcat(s,txt);
        for (j=0;j<nnodes;j++){
            v=lqm[i][j];
            if(i!=j) {
            	if (v<10) sprintf(s1,"  %d",v);
            	else if (v<100) sprintf(s1," %d",v);
            	else  sprintf(s1,"%d",v);
            } else{
            	sprintf(s1,"  -");
            }
            strcat(s,s1);
            strcat(s," ");
        }
      	strcat(s,"\n");
     }
}

void wmp_print_lqm(char ** lqm, char * s, int nnodes){
	char txt[1000];
	lqmToString(lqm,txt, nnodes);
	WMP_MSG(stderr,"%s\n%s\n",s,txt);
}

void wmp_print_lqmn(char * s, int d){
	char txt[1000];
	lqmToString(lqm_get_ptr(),txt,wmpGetNumOfNodes());
	WMP_MSG(stderr,"I'm Node %d, Node: %d: %s\n%s\n",wmpGetNodeId(),d ,s,txt);
}
/* wmp_time */

unsigned long long s_time[10];

void wmp_time_store(int i){
	s_time[i] = getRawActualTimeus();
}

long long wmp_time_diff_us(int i, int j){
	long long diff = (s_time[i] - s_time[j]);
	return diff;
}

int wmp_time_diff_ms(int i, int j){
	long long diff = (s_time[i] - s_time[j]);
	return (int) diff;
}

void wmp_time_print_diff_ms(int i, int j, char * txt){
	WMP_MSG(stderr,"%s: %d ms\n",txt,wmp_time_diff_ms(i,j));
}
void wmp_time_print_diff_us(int i, int j, char * txt){
	WMP_MSG(stderr,"%s: %lld us\n",txt,wmp_time_diff_us(i,j));
}

void wmp_time_subtract(int i){
	long long ts = (getRawActualTimeus() - s_time[i]);
	s_time[i] = ts;
}

void wmp_time_print_us(int i, char * txt){
	WMP_MSG(stderr,"%s: %llu us\n",txt,s_time[i]);
}

void wmp_time_print_ms(int i, char * txt){
   unsigned long long ms = DO_DIV64(s_time[i] , 1000);
	WMP_MSG(stderr,"%s: %llu ms\n",txt,ms);
}

void wmp_time_subtract_and_print_us(int i, char * txt){
	wmp_time_subtract(i);
	wmp_time_print_us(i,txt);
}

void wmp_time_subtract_and_print_ms(int i, char * txt){
	wmp_time_subtract(i);
	wmp_time_print_ms(i,txt);
}

int inline ms_to_us(int n){
	return n*1000;
}
int inline us_to_ms(int n){
	return n/1000;
}


#ifdef NET_DEBUG

static char buffer[1000];
static int buffer_idx = 0;
void wmp_print(const char *format, ...) {
	  va_list args;
	  va_start (args, format);
	  vsprintf (buffer+buffer_idx,format, args);
	  va_end (args);
	  buffer_idx=strlen(buffer);
}

int wmp_print_get_size(void) {
	return strlen(buffer);
}
void wmp_print_reset(void) {
	buffer[0] = 0;
	buffer_idx = 0;
}

int wmp_print_put(wmpFrame * p) {
	int len, tail;
	len = tail = wmp_get_frame_total_lenght(p);
	len = 1500 - len;
	len  = strlen(buffer) > len ? len : strlen(buffer);
	memcpy((((char *) p) + tail), buffer, len);
	//printk(KERN_INFO "%s",buffer);
	wmp_print_reset();
	return len;
}

int inline ms_to_us(int n){
	return n*1000;
}
int inline us_to_ms(int n){
	return n/1000;
}
#else
void wmp_print(const char *format, ...) {
}

int wmp_print_get_size(void) {
	return 0;
}
void wmp_print_reset(void) {
}

int wmp_print_put(wmpFrame * p) {
	return 0;
}

#endif
