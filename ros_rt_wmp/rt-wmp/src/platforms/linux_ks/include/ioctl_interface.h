/*-------------------------------------------------------------------------
 *--------------------------- RT-WMP IP INTERFACE -------------------------
 *-------------------------------------------------------------------------
 *
 * File: ioctl_interface.h
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

#ifndef _PASARELA_INTERFACE_H_
#define _PASARELA_INTERFACE_H_

#include <linux/if.h>
#include "core/interface/Msg.h"

/* Private ioctl's */
#define SIO_TRANSMISSION         (SIOCDEVPRIVATE + 0)
#define SIO_NODEINFO             (SIOCDEVPRIVATE + 1)
#define SIO_GETLATESTLQM         (SIOCDEVPRIVATE + 2)
#define SIO_NETWORKCONNECTED     (SIOCDEVPRIVATE + 3)
#define SIO_QUEUEACTIONS         (SIOCDEVPRIVATE + 4)
#define SIO_QUEUEELEMSINFO       (SIOCDEVPRIVATE + 5)
#define SIO_RTWMPSETGET          (SIOCDEVPRIVATE + 6)
#define SIO_RTWMPPLUGIN          (SIOCDEVPRIVATE + 7)
#define SIO_GETLATESTDISTANCE   (SIOCDEVPRIVATE + 8)

/* Data to use with SIO_TRANSMISION */
typedef enum {PUSH, PUSH_DATA, POP_DATA,POP_DATA_TIMEOUT, POP, TIMEDPOP, NONBLOCKINGPOP} tpTransmission;
typedef struct {
   tpTransmission type;
   Msg *m;
   unsigned int port;
   unsigned int dest;
   unsigned char source;
   unsigned int size;
   char priority;
   int timeout_ms;            // Only for TIMEDPOP
   int ret;
   char * data;
} tpTransmissionInfo;

/* Data to use with SIO_NODEINFO */
typedef enum {NODEID, NUMOFNODES} tpNodeI;
typedef struct {
   tpNodeI type;
   char ret;
} tpNodeInfo;

/* Data to use with SIO_NETWORKCONNECTED */
typedef enum {BLOCKING, NONBLOCKING} tpNetworkConnected;
typedef struct {
   tpNetworkConnected type;
   int timeout;               // Only for BLOCKING 
   int ret;
} tpNetworkConnectedInfo;

/* Data to use with SIO_QUEUEACTIONS */
// THE ORDER OF THE ELEMENTS IN THIS ENUM IS IMPORTANT
typedef enum { REMOVETXMSG, GETCPUDELAY, GETTIMEOUT, GETWCMULT, GETRATE,
               SETCPUDELAY, SETTIMEOUT, SETWCMULT, SETRATE} tpQueueAction;
typedef struct {
   tpQueueAction queueAction;
   int ival;                  // For CPUDELAY and TIMEOUT
   int fval;                // For WCMULT and RATE
   int ret;                   // For REMOVETXMSG
} tpQueueActionInfo;

/* Data to use with SIO_QUEUEELEMSINFO */
typedef enum {NUMOFFREEPOSITIONSTX, NUMOFELEMSTX, NUMOFELEMSRX} tpGetQueueElems;
typedef struct {
   tpGetQueueElems type;
   int ret;
} tpGetQueueElemsInfo;

/* Data to use with SIO_RTWMPSETGET */
// THE ORDER OF THE ELEMENTS IN THIS ENUM IS IMPORTANT
typedef enum { NETIT, GETMTU, GETAS, SERIAL, LOOPID, GINSTANCEID,
               GPRIMBASEDROUTING, GMESSAGERESCHEDULE, GFLOWCONTROL, SETAS,
               SINSTANCEID, SPRIMBASEDROUTING, SMESSAGERESCHEDULE, SFLOWCONTROL} tpRTWMPSetGet;
typedef struct {
   tpRTWMPSetGet type;
   union {
      int netIT;
      unsigned int mtu;
      int activeSearch;
      unsigned int serial;
      unsigned int loopId;
      short instanceId;
      int primBasedRouting;
      int messageReschedule;
      int flowControl;
   };
} tpRTWMPSetGetInfo;

/* Data to use with SIO_RTWMPPLUGIN */
typedef enum {QOS, BC_PLUS} tpPlugin;

/* Data to use with SIO_BROADCAST */
typedef enum {PUSHBC, POPBC, TIMEDPOPBC, ELEMSINRXBC, ELEMSINTXBC, FREEPOSINTXBC} tpBCAcc;
typedef struct {
   tpBCAcc type;
   Msg *m;
   int timeout_ms;            // Only for TIMEDPOPBC
   int ret;
} tpBCInfo;

/* Data to use with SIO_QOS */
typedef enum {PUSHQOS, POPQOS, TIMEDPOPQOS, FREEPOSQOS} tpQoSAcc;
typedef struct {
   tpQoSAcc type;
   Msg *m;
   int timeout_ms;            // Only for TIMEDPOPQOS
   int ret;
} tpQoSInfo;

/* Union passed as data when calling to ioctl() */
typedef union {
   tpTransmissionInfo transmissionInfo;
   tpNodeInfo nodeInfo;
   char *lqm;
   tpNetworkConnectedInfo networkConnected;
   tpQueueActionInfo queueAction;
   tpGetQueueElemsInfo queueElemsInfo;
   tpRTWMPSetGetInfo RTWMPSetGetInfo;
   tpPlugin RTWMPplugin;

   tpBCInfo BCInfo;
   tpQoSInfo QoSInfo;
} tpIOCTL_data;

#endif
