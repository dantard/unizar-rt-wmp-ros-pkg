/*------------------------------------------------------------------------
 *---------------------           RT-WMP              --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: ./src/config.h.in.h                                             
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

#ifndef SUPPORT_H_
#define SUPPORT_H_

/*** WARNING 
* Changes to this file are lost in every ./configure execution
* Modify wmp_config.h.in, instead
*/

/*** GENERAL DEFINES */

//#define WMP_USE_FRAME_COMPRESSION
//#define WMP_USE_MESSAGE_COMPRESSION

/* Descarta uno de cada RANDOM_MESSAGE_DISCARD (mediamente) mensajes recibidos (no envia ack ni encola los
 * mensajes (con valores de 1 y 0 esta disabilitado) */
#define RANDOM_MESSAGE_DISCARD 1


/*** ENHANCEMENTS */

/* Force a worst case loop if a node did not send its LQM  FORCE_WC_LIMIT times (0 disables it) */
//#define FORCE_WC_PAP
#define FORCE_WC_LIMIT 0

/* Define how many times nodes have to search 'desaparecidos' before stopping */
#define ACTIVE_SEARCH_COUNT 5

/* Search nodes continuosly when lost*/
#define AGGRESSIVE

/*** PLUGINS ***/
//#define LOGGER_USE_GZ

#define NO_DEBUG             0
#define CORE				 1
#define LQM					 2
#define QUEUES				 4
#define DIJKSTRA			 8
#define FRAME_COMPRESS		 16
#define ML					 32
#define PRIM				 64
#define WMP_COM				 128
#define LL_COM				 256

#define PLUGINS_DRIVER		 512
#define PLUGIN_FAKE LQM      1024
#define PLUGIN_MQ		     2048
#define PLUGIN_QOS			 4096
#define PLUGIN_LOGGER   	 8192
#define PLUGIN_BC_COMMON  	 16384

#define WMP_DEBUG_LEVEL		NO_DEBUG

/*** AUTOMAKE DEFINES */

#define QOS_SUPPORT_DISABLED

#define WMP_PLATFORM_linux_ks
#define WMP_LLCOM_mac80211_mod
#define WMP_ROUTING_basic

#endif

