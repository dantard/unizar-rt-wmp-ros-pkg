/*------------------------------------------------------------------------
 *---------------------           RT-WMP              --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: ./src/core/interface/Msg.h
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

#ifndef MSG__H
#define MSG__H

#define MTU					1500

typedef struct{
	short deadline; /**< Dealine for QoS Messages */
} qos_t;

typedef struct{
	short bin_size; /**< Bin size for Simultaneous multicast */
} mbc_t;

/* private data used internally */
typedef	struct {
		unsigned char compressed;
		unsigned int offset;
		unsigned short laxity;
		unsigned int univocal_id;
		int rescheduled;
		int flow_control;
		char type;
		short part_id;
		unsigned short msg_hash;
	} priv_t;

typedef struct {
	unsigned int len;          /**< Length of the message   */
	char port;          /**< Port of the message     */
	signed char priority;      /**< Priority of the message */
	unsigned char src;   /**< Source of the message   */
	unsigned int dest;  /**< Destination of the message   */
	union {
		qos_t qos;  /**< QoS relates, see struct   */
		mbc_t mbc;  /**< Multicast relates, see struct   */
	};
	char data[MTU];     /** Payload of the message   */
	priv_t priv;        /** Private data, used internally */
} Msg;

#endif

