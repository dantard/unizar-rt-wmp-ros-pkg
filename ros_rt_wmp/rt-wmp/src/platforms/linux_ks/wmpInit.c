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
#include "core/include/wmp_misc.h"

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

	while (fgets(line, 256, f) != NULL) {
		if (line[0] < 65 || line[0] > 90) {
			continue;
		}
		sscanf(line, "%s %s", param, val);

		exists = apply_config(status, param, val);

		if (exists) {
			printk		(KERN_INFO "READ OPTION: %s = %s\n", param, val);
		} else {
			printk(KERN_INFO "*** UKNOWN OPTION %s = %s\n", param, val);
		}
	}
	printk(KERN_INFO "Done.\n");
	filp_close(f,0);
	return 1;
}
