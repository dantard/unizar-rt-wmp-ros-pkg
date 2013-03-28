/*------------------------------------------------------------------------
 *---------------------           RT-WMP              --------------------
 *------------------------------------------------------------------------
 *
 *
 *
 *  File: ./src/platforms/linux_ks/hwi/ath5k_raw/ll_com.c
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

#include "config/compiler.h"
#include "core/include/definitions.h"
#include "core/include/frames.h"

#include "core/interface/wmp_interface.h"
#include "core/include/wmp_misc.h"
#include "core/include/ml_com.h"

#include <linux/if.h>
#include <linux/if_packet.h>
#include <linux/if_ether.h>
#include <linux/if_ether.h>

#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <net/mac80211.h>
#include <net/ieee80211_radiotap.h>



int ieee80211_rtwmp_init(char * dev_name);
int ieee80211_rtwmp_timed_receive(char * buff, rxInfo * ret, int ms);
void ieee80211_rtwmp_finish(void);
int ieee80211_rtwmp_send(char * buff, int size);


//static struct net_device *interfaz;
static char devname[20];
static char param[256], val[256];

static short freq;
static short rate;
static unsigned char txpower_dbm;
static int antenna_mode;

int readllcfg(void) {
	struct file *f;
	char line[256];
	int exists;

	/* Set default values */
	freq = 2412;
	rate = 110;
	txpower_dbm = 20;
	antenna_mode = 1;

	f = filp_open("/etc/rt-wmp/linux_ks-mac80211_mod.ll", O_RDONLY, 0);

	if (!IS_ERR(f)) {
		WMP_MSG(stderr, "Reading Low Level Configuration file (/etc/rt-wmp/linux_ks-mac80211_mod.ll)... \n");
		while (fgets(line, 256, f) != NULL) {
			if (line[0] < 65 || line[0] > 90) {
				continue;
			}
			sscanf(line, "%s %s", param, val);
			exists = 0;
			if (strcmp(param, "DEVICE") == 0) {
				strcpy(devname, val);
				exists = 1;
			}
			else if (strcmp(param, "FREQ") == 0) {
				freq = atoi(val);
				exists = 1;
			} else if (strcmp(param, "RATE") == 0) {
				rate = atoi(val);
				exists = 1;
			} else if (strcmp(param, "TXPOWER_DBM") == 0) {
				txpower_dbm = atoi(val);
				exists = 1;
			}
			if (exists) {
				WMP_MSG(stderr, "READ OPTION: %s = %s\n", param, val);
			} else {
				WMP_MSG(stderr, "WARNING ::: UKNOWN OPTION %s\n", param);
			}
		}
		WMP_MSG(stderr, "Done.\n");
		filp_close(f, 0);
	} else{
		WMP_MSG(stderr, "File /etc/rt-wmp/linux_ks-mac80211_mod.ll not found, using default values.\n");
	}
	return 0;
}

void closeLowLevelCom(void) {
	ieee80211_rtwmp_finish();
}

int configure_interface(struct net_device *dev) {
	return 1;
}

int initLowLevelCom(void) {
	char name[10];

	//XXX: To be implemented
	// ieee80211_rtwmp_configure_interface(freq, rate, txpower_dbm);

	sprintf(name,"%s","wlan");
	if (ieee80211_rtwmp_init(name)){
		WMP_ERROR(stderr, "Low level com init ok.\n");
		return 1;
	}else{
		return 0;
	}
}

int llsend(char * f, int size) {
	return ieee80211_rtwmp_send(f, size);
}

rxInfo llreceive(char *f, int timeout) {
	rxInfo ret;
	memset(&ret, 0, sizeof(ret));
	if (ieee80211_rtwmp_timed_receive(f, &ret, timeout)) {
		ret.rssi = 96 + ret.rssi;
		ret.rssi = ret.rssi > 100 ? 100 : ret.rssi;
		ret.rssi = ret.rssi > 0 ? ret.rssi  : 1;
	}
	return ret;
}


