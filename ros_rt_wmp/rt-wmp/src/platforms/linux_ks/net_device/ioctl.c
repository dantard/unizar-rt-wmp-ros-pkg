/*-------------------------------------------------------------------------
 *--------------------------- RT-WMP IP INTERFACE -------------------------
 *-------------------------------------------------------------------------
 *
 * File: ioctl.c
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

#include "ioctl.h"
#include "../config/compiler.h"
#include <linux/module.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <asm/uaccess.h>
#include "core/interface/wmp_interface.h"

int ioctl_node_info(tpNodeInfo __user *pinfo) {
	int ret = 0;
	tpNodeInfo info;

	ret = copy_from_user(&info, pinfo, sizeof(tpNodeInfo));
	if (ret)
		return ret;

	switch (info.type) {
	case NODEID:
		info.ret = wmpGetNodeId();
		break;
	case NUMOFNODES:
		info.ret = wmpGetNumOfNodes();
		break;
	}

	ret = copy_to_user(pinfo, &info, sizeof(tpNodeInfo));

	return ret;
}

int ioctl_lqm(char __user *plqm) {
	char lqm[wmpGetNumOfNodes() * wmpGetNumOfNodes()];
	int size = wmpGetLatestLQM(lqm);
	return copy_to_user(plqm, lqm, size);
}

int ioctl_network_connected(tpNetworkConnectedInfo __user *pinfo) {
	int ret = 0;
	tpNetworkConnectedInfo info;

	ret = copy_from_user(&info, pinfo, sizeof(tpNetworkConnectedInfo));
	if (ret)
		return ret;

	if (info.type == BLOCKING) {
		info.ret = wmpIsNetworkConnectedBlocking(info.timeout);
	} else {
		info.ret = wmpIsNetworkConnected();
	}
	ret = copy_to_user(pinfo, &info, sizeof(tpNetworkConnectedInfo));

	return ret;
}

int ioctl_queue_actions(tpQueueActionInfo __user *pinfo) {
	int ret = 0;
	tpQueueActionInfo info;

	ret = copy_from_user(&info, pinfo, sizeof(tpQueueActionInfo));
	if (ret)
		return ret;

	switch (info.queueAction) {
	case REMOVETXMSG:
		info.ret = wmp_queue_tx_remove_head();
		break;
	case GETCPUDELAY:
		info.ival = wmpGetCpuDelay();
		break;
	case GETTIMEOUT:
		info.ival = wmpGetTimeout();
		break;
	case GETWCMULT:
		//XXX: info.fval = wmpGetWCMult();
		break;
	case GETRATE:
		//XXX: info.fval = wmpGetRate();
		break;
	case SETCPUDELAY:
		wmpSetCpuDelay(info.ival);
		break;
	case SETTIMEOUT:
		wmpSetTimeout(info.ival);
		break;
	case SETWCMULT:
		//XXX: wmpSetWCMult(info.fval);
		break;
	case SETRATE:
		//XXX: wmpSetRate(info.fval);
		break;
	}
	/* If the operation is a GET, copy the result to user space */
	if (info.queueAction < SETCPUDELAY) {
		ret = copy_to_user(pinfo, &info, sizeof(tpQueueActionInfo));
	}

	return ret;
}

int ioctl_queue_elems_info(tpGetQueueElemsInfo __user *pinfo) {
	int ret = 0;
	tpGetQueueElemsInfo info;

	ret = copy_from_user(&info, pinfo, sizeof(tpGetQueueElemsInfo));
	if (ret)
		return ret;

	switch (info.type) {
	case NUMOFFREEPOSITIONSTX:
		info.ret = wmp_queue_tx_get_room();
		break;
	case NUMOFELEMSTX:
		info.ret = wmpGetNumOfElementsInTXQueue();
		break;
	case NUMOFELEMSRX:
		info.ret = wmpGetNumOfElementsInRXQueue(0);
		break;
	}
	ret = copy_to_user(pinfo, &info, sizeof(tpGetQueueElemsInfo));

	return ret;
}

int ioctl_setget(tpRTWMPSetGetInfo __user *pinfo) {
	int ret = 0;
	tpRTWMPSetGetInfo info;

	ret = copy_from_user(&info, pinfo, sizeof(tpRTWMPSetGetInfo));
	if (ret)
		return ret;

	switch (info.type) {
	case NETIT:
		info.netIT = wmpGetNetIT();
		break;
	case GETMTU:
		info.mtu = wmpGetMTU();
		break;
	case GETAS:
		info.activeSearch = wmpGetActiveSearch();
		break;
	case SERIAL:
		info.serial = wmpGetSerial();
		break;
	case LOOPID:
		info.loopId = wmpGetLoopId();
		break;
	case GINSTANCEID:
		info.instanceId = wmpGetInstanceId();
		break;
	case GPRIMBASEDROUTING:
		info.primBasedRouting = wmpGetPrimBasedRouting();
		break;
	case GMESSAGERESCHEDULE:
		info.messageReschedule = wmpGetMessageReschedule();
		break;
	case GFLOWCONTROL:
		info.flowControl = wmpGetFlowControl();
		break;

	case SETAS:
		wmpSetActiveSearch(info.activeSearch);
		break;
	case SINSTANCEID:
		wmpSetInstanceId(info.instanceId);
		break;
	case SPRIMBASEDROUTING:
		wmpSetPrimBasedRouting(info.primBasedRouting);
		break;
	case SMESSAGERESCHEDULE:
		wmpSetMessageReschedule(info.messageReschedule);
		break;
	case SFLOWCONTROL:
		wmpSetFlowControl(info.flowControl);
		break;
	}

	/* If the operation is a GET, copy the result to user space */
	if (info.type < SETAS) {
		ret = copy_to_user(pinfo, &info, sizeof(tpRTWMPSetGetInfo));
	}

	return ret;
}
