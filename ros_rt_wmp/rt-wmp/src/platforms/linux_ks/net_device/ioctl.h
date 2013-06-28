/*-------------------------------------------------------------------------
 *--------------------------- RT-WMP IP INTERFACE -------------------------
 *-------------------------------------------------------------------------
 *
 * File: ioctl.h
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

#ifndef _IOCTL_H_
#define _IOCTL_H_

#include "../include/ioctl_interface.h"

int ioctl_node_info(tpNodeInfo __user *pinfo);

int ioctl_lqm(char __user *plqm);

int ioctl_distances(char __user *plqm);

int ioctl_network_connected(tpNetworkConnectedInfo __user *pinfo);

int ioctl_queue_actions(tpQueueActionInfo __user *pinfo);

int ioctl_queue_elems_info(tpGetQueueElemsInfo __user *pinfo);

int ioctl_setget(tpRTWMPSetGetInfo __user *pinfo);


int ioctl_broadcast(tpBCInfo __user *pinfo);

int ioctl_qos(tpQoSInfo __user *pinfo);

#endif
