/*------------------------------------------------------------------------
 *---------------------           RT-WMP              --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: ./src/core/frame_compress.c
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

#include "wmp_config.h"
#include "config/compiler.h"

#include "include/ml_com.h"
#include "include/ll_com.h"

char zFrame[2500];

#ifdef WMP_USE_FRAME_COMPRESSION
#include <zlib.h>

long long max = 0;

int cmp_send(char * f, int size){
	uLongf len = 2500;
	WMP_DBG(FRAME_COMPRESS,"sending %d bytes\n", size);

	unsigned long long ts1 = getRawActualTimeus();
	int ret = compress(zFrame,&len,f,size);
	unsigned long long ts2 = getRawActualTimeus();
	long long ts = ts2 - ts1;
	if (ts > max) max = ts;
	WMP_DBG(FRAME_COMPRESS,"cmp_time = %lld max:%lld \n",ts,max);

	if (ret != Z_OK){
		WMP_DBG(FRAME_COMPRESS,"cmp err\n");
		return -1;
	}
	WMP_DBG(FRAME_COMPRESS,"sending %d bytes (compressed) \n", len);
	return llpsend(zFrame,len,ZWMP_TYPE_FIELD);
}

rxInfo  cmp_receive(char * f,  int timeout){
	rxInfo rxi = llreceive(zFrame,timeout);
	if (rxi.proto == 0x6970){
		WMP_DBG(FRAME_COMPRESS,"recvd %d bytes proto: %x\n", rxi.size, rxi.proto);
		uLongf len = 2500;
		unsigned long long ts1 = getRawActualTimeus();
		int ret = uncompress(f,&len,zFrame,rxi.size);
		unsigned long long ts2 = getRawActualTimeus();
		long long ts = ts2 -ts1;

		WMP_DBG(FRAME_COMPRESS,"decmp_time = %lld\n",ts);

		if (ret != Z_OK){
			WMP_DBG(FRAME_COMPRESS,"uncmp err\n");
			rxi.error = 1;
			return rxi;
		}
		rxi.size = len;
		WMP_DBG(FRAME_COMPRESS,"returning %d bytes\n",len);
	}else{
		memcpy(f,zFrame,rxi.size);
	}
	return rxi;
}
#else

int cmp_send(char * f, int size){
	return llsend(f,size);
}

rxInfo cmp_receive(char * f,  int timeout){
	return llreceive(f,timeout);
}

#endif




