/*-------------------------------------------------------------------------
 *--------------------------- RT-WMP IP INTERFACE -------------------------
 *-------------------------------------------------------------------------
 *
 * File: conf.c
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

#include "config/compiler.h"
#include "core/include/global.h"

#include <linux/module.h>
#include <linux/kernel.h>
#include <asm/uaccess.h>
#include "conf.h"

/**
 *  If there is space left in 'portConf' for another port, stores the
 *  configuration (priority and type of traffic) for the port 'number'
 *  If 'number' is 0, the configuration is used for every port
 */
void asign_port (tpPortConf* portConf, u16 number, signed char priority, tpTraffic traffic) {
   if (number == 0) {             // Global configuration
      portConf->count = -1;
      portConf->port[0].number = number;
      portConf->port[0].priority = priority;
      portConf->port[0].traffic = traffic;
   } else {
      /* If there is no global configuration and space for another port,
         store the configuration for the given port number */
      if(portConf->count != -1 && portConf->count < NUM_PORTS) {
         portConf->port[portConf->count].number = number;
         portConf->port[portConf->count].priority = priority;
         portConf->port[portConf->count].traffic = traffic;
         portConf->count++;
      }
   }
}

/**
 *  Stores 'priority' and 'traffic' as configuration in 'portConf'
 */
void asignConfICMP (tpPortConfICMP* portConf, signed char priority, tpTraffic traffic) {
   if (portConf->count == 0) {
      portConf->count++;
      portConf->priority = priority;
      portConf->traffic = traffic;
   } else {
      printk(KERN_INFO "ICMP already has a configuration.");
   }
}

int readConfig(tpConfig *conf) {
   struct file *f;
   char proto[6], dir[5], traf[10], line[256];
   tpTraffic traffic;
   int port, priority;
   int exists, ret = 0;

   /* Initializations */
   conf->udp_in.count=0;
   conf->udp_out.count=0;
   conf->tcp_in.count=0;
   conf->tcp_out.count=0;
   conf->icmp.count=0;

   /* Open thee file */
   f = filp_open("/etc/rt-wmp/interface.cfg", O_RDONLY, 0);
   if (IS_ERR(f)) {
      printk(KERN_INFO "File /etc/rt-wmp/interface.cfg not found...\n");
      return 0;
   }

   /* Read the configuration */
   printk(KERN_INFO "Reading /etc/rt-wmp/interface.cfg...\n");
   while (fgets(line,256,f) != NULL) {
      if (line[0]<65 || line[0]>90){
         continue;
      }

      if(sscanf(line,"%s %s %d %d %s",proto,dir, &port, &priority, traf) != 5){
         printk(KERN_INFO "MALFORMED: %s\n",line);
         continue;
      }

      if (strcmp(traf, "NORMAL") == 0) {
         traffic = OTHER;
      } else if (strcmp(traf, "QoS") == 0) {
         traffic = QoS;
      } else {
         printk(KERN_INFO "Wrong type of traffic: %s\n",traf);
         continue;
      }

      exists = 0;
      if (strcmp(proto, "UDP") == 0) {
         if (strcmp(dir, "FROM") == 0) {
            asign_port(&conf->udp_in, port, priority, traffic);
            exists = 1;
         } else if (strcmp(dir, "TO") == 0) {
            asign_port(&conf->udp_out, port, priority, traffic);
            exists = 1;
         } else if (strcmp(dir, "BOTH") == 0) {
            asign_port(&conf->udp_in, port, priority, traffic);
            asign_port(&conf->udp_out, port, priority, traffic);
            exists = 1;
         } else {
            printk(KERN_INFO "*** UNKNOWN DIRECTION %s\n", dir);
         }
      } else if (strcmp(proto, "TCP") == 0) {
         if (strcmp(dir, "FROM") == 0) {
            asign_port(&conf->tcp_in, port, priority, traffic);
            exists = 1;
         } else if (strcmp(dir, "TO") == 0) {
            asign_port(&conf->tcp_out, port, priority, traffic);
            exists = 1;
         } else if (strcmp(dir, "BOTH") == 0) {
            asign_port(&conf->tcp_in, port, priority, traffic);
            asign_port(&conf->tcp_out, port, priority, traffic);
            exists = 1;
         } else {
            printk(KERN_INFO "*** UNKNOWN DIRECTION %s\n", dir);
         }
      } else if (strcmp(proto, "ICMP") == 0) {
         asignConfICMP(&conf->icmp, priority, traffic);
         exists = 1;
      } else {
         printk(KERN_INFO "*** UNKNOWN PROTOCOL %s\n", proto);
      }

      if (exists) {
         printk(KERN_INFO "READ: %s %s %d %d %s\n", proto, dir, port, priority, traf);
      }
      ret = (ret || exists);
   }
   printk(KERN_INFO "Done.\n");

   /* Close the file and return */
   filp_close(f,0);

   // Will be TRUE if one or more ports have been read
   return ret;
}

int __getPortConf (tpPortConf* portConf, u16 port, signed char *priority, tpTraffic *traffic) {
   int i, ret = 0;
   if(portConf->count == -1) {            // Same config for every port
      *priority = portConf->port[0].priority;
      *traffic = portConf->port[0].traffic;
      ret = 1;
   } else {
      for(i=0; i<portConf->count && !ret; i++){
         ret = (port == portConf->port[i].number);
         if(ret) {
            *priority = portConf->port[i].priority;
            *traffic = portConf->port[i].traffic;
         }
      }
   }
   return ret;
}

int getPortConf (tpConfig *conf, tpProto proto, tpDir dir, u16 port, signed char *priority, tpTraffic *traffic){
   int ret = 0;

   switch(proto){
      case UDP:
         if (dir == FROM) {
            ret = __getPortConf(&conf->udp_in, port, priority, traffic);
         } else {
            ret = __getPortConf(&conf->udp_out, port, priority, traffic);
         }
         break;
      case TCP:
         if (dir == FROM) {
            ret = __getPortConf(&conf->tcp_in, port, priority, traffic);
         } else {
            ret = __getPortConf(&conf->tcp_out, port, priority, traffic);
         }
         break;
   }

   return ret;
}

int getConfICMP (tpConfig* conf, signed char *priority, tpTraffic *traffic) {
   int ret = 0;
   if(conf->icmp.count > 0) {
      *priority = conf->icmp.priority;
      *traffic = conf->icmp.traffic;
      ret = 1;
   }
   return ret;
}
