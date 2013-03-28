/*------------------------------------------------------------------------
 *---------------------           RT-WMP              --------------------
 *------------------------------------------------------------------------
 *                                                         
 *
 *
 *  File: ./src/platforms/linux_ks/wmpInit.c
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
#include "core/include/global.h"

#include <linux/module.h>
#include <linux/kernel.h>
#include <asm/uaccess.h>

int wmpReadConfiguration(Status * status) {
	struct file *f;
	char param[30], val[20], line[256];
   int exists;

	f = filp_open("/etc/rt-wmp/rt-wmp.cfg", O_RDONLY, 0);
	if (IS_ERR(f)) {
		printk(KERN_INFO "File /etc/rt-wmp/rt-wmp.cfg not found...\n");
      printk(KERN_INFO "Using default values (timeout: %d).\n", status->TIMEOUT);
		return -1;
	}
   printk(KERN_INFO "Reading /etc/rt-wmp/rt-wmp.cfg...\n");
	while (fgets(line,256,f) != NULL) {
		if (line[0]<65 || line[0]>90){
			continue;
		}
		sscanf(line,"%s %s",param,val);
		exists = 0;
		if (strcmp(param, "WMP_ADDRESS") == 0) {
			status->id = atoi(val);
			exists = 1;
		} else if (strcmp(param, "N_NODES") == 0) {
			status->N_NODES = atoi(val);
			exists = 1;
		} else if (strcmp(param, "QUIET") == 0) {
			status->quiet = atoi(val);
			exists = 1;
		} else if (strcmp(param, "MAX_RSSI") == 0) {
			status->max_rssi = atoi(val);
			exists = 1;
		} else if (strcmp(param, "TIMEOUT") == 0) {
			exists = 1;
			status->TIMEOUT = atoi(val);
		} else if (strcmp(param, "HOLD_TIME") == 0) {
			status->hold_time = atoi(val);
			exists = 1;
		} else if (strcmp(param, "MAX_PER_NODE_RETRIES") == 0) {
			status->maxPerNodeRetries = atoi(val);
			exists = 1;
		} else if (strcmp(param, "MAX_TOTAL_RETRIES") == 0) {
			status->maxTotalRetries = atoi(val);
			exists = 1;
		} else if (strcmp(param, "PER_HOP_DELAY") == 0) {
			status->perHopDelay = atoi(val);
			exists = 1;
		} else if (strcmp(param, "MOBILE_AVERAGE_ELEMENTS") == 0) {
			status->mobile_average_elements = atoi(val);
			exists = 1;
		} else if (strcmp(param, "QUEUE_ELEMENTS") == 0) {
			status->queue_elements = atoi(val);
			exists = 1;
		} else if (strcmp(param, "RSSI_RISING_FACTOR") == 0) {
			status->rssi_rising_factor=atoi(val);
			exists = 1;
		} else if (strcmp(param, "NET_ID") == 0) {
			status->net_id = atoi(val);
			exists = 1;
		} else if (strcmp(param, "ETT_MULT") == 0){
         ATOF(val, &(status->ett_mult));
			exists = 1;
		} else if (strcmp(param, "ETT_MAX") == 0){
         ATOF(val, &(status->ett_max));
			exists = 1;
		} else if (strcmp(param, "USE_ETT") == 0){
			status->use_ett = atoi(val);
			exists = 1;
		}

		if (exists) {
			printk(KERN_INFO "READ OPTION: %s = %s\n", param, val);
		} else {
			printk(KERN_INFO "*** UKNOWN OPTION %s = %s\n", param, val);
		}
	}
	printk(KERN_INFO "Done.\n");
   filp_close(f,0);
	return 1;
}
