/*------------------------------------------------------------------------
 *---------------------           RT-WMP              --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: ./src/core/ml_com.c
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

#include "core/include/frames.h"
#include "core/include/definitions.h"
#include "core/interface/wmp_interface.h"
#include "core/include/wmp_misc.h"
#include "core/include/ml_com.h"
#include "core/include/ll_com.h"
#include "core/include/frame_compress.h"
#include "core/include/wmp_utils.h"
#include "core/include/global.h"

static int wack = 0;
static int filter_type = 0, filter_param = 0;

void closeMiddleLevelCom() {
	closeLowLevelCom();
}

int initMiddleLevelCom() {
	return initLowLevelCom();
}
void ml_set_ett(int _use_ett, int _ett_mult, int _ett_max) {
	status.use_ett = _use_ett;
}

int ml_calculate_duration(int rate, int size) {
	return 1;
}

/* FILTER */
void ml_set_filter(char * ftype, int param){
	filter_param = param;
	if (strcmp(ftype,"chain") == 0){
		filter_type = 1;
	}else if (strcmp(ftype,"star") == 0){
		filter_type = 2;
	}
}

void wmpForceTopology(char * name, int param){
	ml_set_filter(name, param);
}

static int ml_filter(char from){
	switch (filter_type){
	case 1:{ /* chain */
		if (abs(from-wmpGetNodeId())>1){
			return 1;
		}else{
			return 0;
		}
	}
	case 2:{ /* star */
		if (wmpGetNodeId()==filter_param){
			return 0;
		}else{
			if (from == filter_param){
				return 0;
			}
		}
		return 1;
	}
	default:
		return 0;
	}
}


/* RX_ERROR */
static int rx_error_type = 0, last_discarded = 0, rx_error_param = 1,last_discarded_type = 0;
void ml_set_rx_error(char * ftype, int param){
	rx_error_param = param;
	if (strcmp(ftype,"all") == 0){
		rx_error_type = 1;
	}else if (strcmp(ftype,"messages") == 0){
		rx_error_type = 2;
	}else if (strcmp(ftype,"unconfirm") == 0){
		rx_error_type = 3;
	}
}

void wmpSetRxError(char * name, int rate){
	if (rate == 0){
		rx_error_param = 1;
	}
	ml_set_rx_error(name, rate);
}

static int ml_rx_error(wmpFrame * p){
	int discard = 0;
	if (rx_error_type == 0 || rx_error_param == 0){
		return 0;
	}

//	if (p->hdr.to != wmpGetNodeId()){
//		return 0;
//	}

//	if (p->hdr.serial == last_discarded && p->hdr.type == last_discarded_type){
//		return 1;
//	}
	switch (rx_error_type){
	case 1:
		discard = ((rand() % rx_error_param) == 0);
		break;
	case 2:
		if (p->hdr.type == MESSAGE){
			discard = ((rand() % rx_error_param) == 0);
		}
		break;
	case 3:
		discard = 0;
		if ((rand() % rx_error_param) == 0){
			mBitsUnset(p->hdr.ack,rand() % wmpGetNumOfNodes());
		}
		break;
	}


	if (discard){
		last_discarded = p->hdr.serial;
		last_discarded_type = p->hdr.type;
	}
	return discard;
}

/* TX and RX functions */

int ml_send(wmpFrame * p, int size) {
	int ret = UNDEF;

	if (p->hdr.type != DROP_TOKEN && p->hdr.type != ACK) {
		/* every time I send something, I wait an ack */
		wack = 1;
	}
	p->hdr.net_id = status.net_id;

	ret = cmp_send((char *) p, size);

	return ret;
}

int ml_receive(wmpFrame *f, int timeout) {
	rxInfo ret;
	struct timespec before, after;
	unsigned long long elapsed_ms = 0, tot_elapsed = 0; //, orig_timeout=timeout;

	while (timeout > 0) {
		/* wait TIMEOUT for acknoledwge frame or for net inactivity */
		GETNSTIMEOFDAY(&before);
		ret = cmp_receive((char*) f, timeout);
		GETNSTIMEOFDAY(&after);

		elapsed_ms = wmp_elapsed_ms(&before, &after);
		tot_elapsed+= elapsed_ms;

//      fprintf(stderr,"Elapsed:%llu error:%d protocol:%x size:%d \n",elapsed_ms, ret.error, ret.proto, ret.size);

		if (ret.error) {
			return EXPIRED;
		}

		timeout = (timeout - elapsed_ms) > 0 ? (timeout - elapsed_ms) : 0;


		if (ret.proto == 0x6969 || ret.proto == 0x6970) {

			if (f->hdr.from == status.id) {
				/* discard sent frame if present*/
				continue;
			}

			if (ml_filter(f->hdr.from)){
				//continue;
			}

            if (ml_rx_error(f)){
                continue;
            }

            if (f->hdr.net_id == status.net_id) {
				if (ret.has_lq) {
					f->hdr.rssi = ret.rssi;
					f->hdr.noise = ret.noise;
				}
                return RECEIVE_OK;

            } else {
				WMP_MSG(stderr, "*** Warning ::: Foreign network detected (id:%d instead of id:%d)\n", f->hdr.net_id,status.net_id);
				continue;
			}
		} else{
			continue;
		}
	}

	return EXPIRED;
}



