/*------------------------------------------------------------------------
 *---------------------           RT-WMP              --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: ./src/core/interface/wmp_interface.h
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

#ifndef WMPINTERFACE__H
#define WMPINTERFACE__H

#include <stdarg.h>
#include "wmp_config.h"
#include "Msg.h"



/* Queues*/
/** Enqueue a message in the TX Queue 
*   @param p the message to push
*   @return 1 if success 0 if not (queue full)
*/
int          wmpPush(Msg *p);

/** Dequeue a message from the RX Queue 
*   @param p the message to pop
*   @return time spent in the queue (age)
*/
unsigned int wmpPop(Msg * p); 

/** Dequeue a message from the RX Queue if  already in the queue or enqueued within 
*   timeous_ms milliseconds. If there are not message to dequeue, 
*   returns a negative value
*   @param p the message to pop
*   @param timeout_ms pop timeout in milliseconds
*   @return time spent in the queue (age) if dequeued -1 if timeout expired
*/
int          wmpTimedPop(Msg * p, int timeout_ms);

/** Dequeue a message from the RX Queue if present. 
*   If there are not message to dequeue, 
*   returns a negative value
*   @param p the message to pop
*   @return time spent in the queue (age) if dequeued -1 if no message to pop
*/
int          wmpNonBlockingPop(Msg * p);

#ifdef USE_MESSAGE_COMPRESSION

int wmpZPush(Msg * m);

#endif
/** Returns the WMP address of the local node
@return  the WMP address of the local node 
*/
char wmpGetNodeId(void);

/** Returns the total number of nodes of the network
@return the total number of nodes of the network
*/
char wmpGetNumOfNodes(void);

/** Returns latest lqm in a buffer.
* The Matrix should be scanned with a innested for loop:<br/>
<code> <PRE>
        char lqm[wmpGetNumOfNodes()*wmpGetNumOfNodes()],p;
        wmpGetLatestLQM(lqm);
        p = lqm;
	for (i = 0; i<wmpGetNumOfNodes() ; i++){
		for (j = 0; j<wmpGetNumOfNodes() ; j++){
			element_ij = *p;
			p++;
		};
	};
</PRE><code/>
*	@param lqm an (at least) n x n char buffer (output parameter)
*	@return always 0 
*/
int  wmpGetLatestLQM(char * lqm);

/** Allows knowing if the network is connected (i.e. all the nodes are up and working
* and connected among them) 
* @return 1 if connected 0 if disconnected
*/
int  wmpIsNetworkConnected(void);

/** Allows knowing if the network is connected (i.e. all the nodes are up and working
* and connected among them and if not, blocks the caller thread until connected
* @param timeout_ms specify the maximum wait for connection (if timeout_ms = 0 wait forever)
* @return 1 if connected 0 if disconnected
*/
int  wmpIsNetworkConnectedBlocking(int timeout_ms);

/** Remove the less priority message from the TX queue 
@param p the removed message
*/
int  wmp_queue_tx_remove_head(void);

/** Sets the delay that RT-WMP process waits before transmitting (to leave the processor
to other tasks)
@param val time in milliseconds
*/
void wmpSetCpuDelay(int val);

/** Returns the delay that RT-WMP process waits before transmitting (to leave the processor
to other tasks)
@return delay in milliseconds
*/
int wmpGetCpuDelay(void);

/** Sets the reception timeout
@param val time in milliseconds
*/
void wmpSetTimeout(int val);

/** Returns the reception timeout
@return reception timeout in milliseconds
*/
int wmpGetTimeout(void);

/** Sets multiplier for the worst-case loop of the protocol (used to force worse worst-cases
and increase QoS performances)
@param val multiplier 
*/
void wmpSetWCMult(int val);

/** Returns multiplier for the worst-case loop of the protocol (used to force worse worst-cases
and increase QoS performances)
@return multiplier
*/
int wmpGetWCMult(void);

/** Sets network rate (used to calculate delays in frame-hops)
@param val rate in Mbps
*/
void wmpSetRate(int val);

/** Returns the network rate (used to calculate delays in frame-hops)
@return rate in Mbps
*/
int wmpGetRate(void);

/* USER */


/** Returns the number of free posiitons in the TX queue
@return number of free positions in the TX queue
*/
int wmp_queue_tx_get_room(void);

/** Returns the number of elements in the TX queue
@return number of elements in the TX queue
*/
int wmpGetNumOfElementsInTXQueue(void);

/** Returns the number of elements in the RX queue
@return number of elements in the RX queue
*/
int wmpGetNumOfElementsInRXQueue(int port);

/** Sets up the network
@param node_id specify the WMP address of the local node
@param active_nodes specify the number of nodes in the network
*/


int  wmpSetup(char node_id, char active_nodes);
int wmpSetupList(int _node_id, int _num_nodes, int nparam, ...);

/** Starts the process in foreground (blocking)*/
void wmpRun(void);

/** Starts the process in background (separate thread)*/
void wmpRunBG(void);

/** Stop the process carefully */
void wmpExit(void);

/** Stops the process killing the thread*/
void wmpInmediateExit(void);

/** Avoid node tranmission*/
void wmpSetQuiet(void);

#ifndef __KERNEL__
/** Enable RT-WMP SIGINT control (CTRL^C will show an alive message)*/
void wmpEnableIntControl(void);
#endif

/** Returns the network inactivity timeout
@return the net inactivity timeout
*/
int wmpGetNetIT(void);

/** Returns the Maximum Transmission Unit for USER data
@return the MTU size in bytes
*/
unsigned int wmpGetMTU(void);


/** Set / Unset the active search capability (default:on)
*/

void wmpSetActiveSearch(int val);

/** Returns the status of the active search capability
@return status 1:on 0:off
*/

int wmpGetActiveSearch(void);

void wmpSetInstanceId(short iid);
void wmpSetPrimBasedRouting(int val);

void wmpSetMessageReschedule(int val);

void wmpSetFlowControl(int val);

short wmpGetInstanceId(void);
int wmpGetPrimBasedRouting(void);

int wmpGetMessageReschedule(void);

int wmpGetFlowControl(void);

unsigned int wmpGetSerial(void);
unsigned int wmpGetLoopId(void);

int wmpPushData           (unsigned int port, char  * p, unsigned int   size, unsigned int   dest, signed char priority);
int wmpPopData            (unsigned int port, char ** p, unsigned int * size, unsigned char * src, signed char * priority);
int wmpPopDataTimeout     (unsigned int port, char ** p, unsigned int * size, unsigned char * src, signed char * priority, int to);
int wmpPopDataCopy        (unsigned int port, char  * p, unsigned int * size, unsigned char * src, signed char * priority);
int wmpPopDataTimeoutCopy (unsigned int port, char  * p, unsigned int * size, unsigned char * src, signed char * priority, int to);
void wmpPopDataDone       (int id);
int wmpWaitData(unsigned int port, int to);
void wmpForceTopology(char * name, int parameter);
void wmpSetRxError(char * name, int rate);
int wmpSetTaskMinimumSeparation(int port, int period);
int  wmpIsKernelSpace(void);
void wmpForceBurst(int port);
char lqm_get_val(int i, int j);
#endif

