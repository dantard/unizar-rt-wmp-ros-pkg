/*------------------------------------------------------------------------
 *---------------------           RT-WMP              --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: ./src/core/nstat.c
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

/* X7 X6 X5 X4 X3 X2 X1 X0
* X4 X3 X2 X1 X0 => Node that have to search
* X5 : 1 : BC Reached
* X6 : 1 : Lost
* X7 : 0 Unreached 1: Reached
*/
#include "include/nstat.h"
unsigned char * nstat;

void  init_nstat(int size){
	int i;
	nstat= (unsigned char *) MALLOC(sizeof(char)*size);
	for (i=0;i<size;i++){
		nstat[i]=0;
	}
}

void  free_nstat(){
	FREE(nstat);
}

/* BIT FUNCTIONS */
void nstat_set_bit(unsigned char *a, unsigned char n){
   *a |= (1 << n);
}
int nstat_test_bit(unsigned char e, unsigned char n){
   e = e & (1 << n);
   return (e>0);
}

void nstat_clear_bit(unsigned char *b, unsigned char n){
   *b &= ~(1 << n);
}
void nstat_set_if_set(unsigned char src, unsigned char *dst,unsigned char n){
	if (nstat_test_bit(src,n)){
		nstat_set_bit(dst,n);
	} else{
		nstat_clear_bit(dst,n);
	}
}

/* EXPORTED FUNCTIONS */

void nstat_set_val(int idx, unsigned char id){
   	int i;
   	unsigned char *val=&nstat[idx];
   	unsigned char tmp=*val;
   	*val=id;
   	for (i=5;i<8;i++){
		nstat_set_if_set(tmp,val,i);
   	}
   	val=&id;
}
unsigned char nstat_get_val(int idx){
   	int i;
   	unsigned char val=nstat[idx];
   	for (i=5;i<8;i++){
		nstat_clear_bit(&val,i);
   	}
   	return val;
}
int nstat_isReached(int idx){
   unsigned char val=nstat[idx];
   return nstat_test_bit(val,7);
}
int nstat_isLost(int idx){
   unsigned char val=nstat[idx];
   return nstat_test_bit(val,6);
}
void nstat_clearReached(int idx){
	unsigned char *val=&nstat[idx];
    return nstat_clear_bit(val,7);
}
void  nstat_setReached(int idx){
    unsigned char *val=&nstat[idx];
    nstat_set_bit(val,7);
}
int nstat_hasFailed(int idx){
	unsigned char val=nstat[idx];
    return nstat_test_bit(val,5);
}
void nstat_clearFailed(int idx){
   unsigned char *val=&nstat[idx];
   nstat_clear_bit(val,5);
}
void  nstat_setFailed(int idx){
   unsigned char *val=&nstat[idx];
   nstat_set_bit(val,5);
}
void  nstat_setLost(int idx){
   unsigned char *val=&nstat[idx];
   nstat_set_bit(val,6);
}
void  nstat_clearLost(int idx){
   unsigned char *val=&nstat[idx];
   nstat_clear_bit(val,6);
}

char nstat_get_byte(int idx){
	return (char) nstat[idx];
}
void nstat_set_byte(int idx,char val){
	nstat[idx]=(unsigned char) val;
}





