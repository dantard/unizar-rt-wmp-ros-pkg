/*-------------------------------------------------------------------------
 *--------------------------- RT-WMP IP INTERFACE -------------------------
 *-------------------------------------------------------------------------
 *
 * File: conf.h
 * Authors: Rubén Durán
 *          Danilo Tardioli
 *-------------------------------------------------------------------------
 *  Copyright (C) 2011, Universidad de Zaragoza, SPAIN
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *-----------------------------------------------------------------------*/

#ifndef _CONF_H_
#define _CONF_H_

#define NUM_PORTS 50

#include <asm/uaccess.h>
#include <linux/proc_fs.h>

/* Type of the message to be sent */
typedef enum {QoS, BROADCAST, OTHER, DROP} tpTraffic;

/* Generic configuration structure */
typedef struct  {
    u16         number;
    signed char        priority;
    tpTraffic   traffic;
    struct proc_dir_entry *proc_entry;
 }tpPort;

typedef struct{
   tpPort port[NUM_PORTS];
   short count;
}tpPortConf;

/* Structure for ICMP configuration
   ICMP has no port, so there is just one configuration */
typedef struct{
   signed char        priority;
   tpTraffic   traffic;
   short       count;
}tpPortConfICMP;


/* Global configuration structure */
typedef struct{
   tpPortConf     udp_in;
   tpPortConf     udp_out;
   tpPortConf     tcp_in;
   tpPortConf     tcp_out;
   tpPortConfICMP icmp;
}tpConfig;

typedef enum {UDP, TCP} tpProto;

/* Internal or external port */
typedef enum {FROM, TO} tpDir;


/**
 *  Reads the configuration from the file and stores it in 'conf'
 */
int readConfig(tpConfig *conf);

/**
 *  Returns TRUE only if the given port has a configuration in 'conf' for
 *  the given protocol and the given direction
 *  The priority and the type of traffic are assigned to 'priority' and 'traffic'
 */
int getPortConf (tpConfig *conf, tpProto proto, tpDir dir, u16 port, signed char *priority, tpTraffic *traffic);

/**
 *  Returns TRUE only if there is a configuration for ICMP in 'conf'
 *  The priority and the type of traffic for ICMP are assigned to 'priority' and 'traffic'
 */
int getConfICMP (tpConfig* conf, signed char *priority, tpTraffic *traffic);

int conf_init_proc(void);
int conf_close_proc(tpConfig *conf);
struct proc_dir_entry * get_proc_root(void);

#endif
