/*------------------------------------------------------------------------
 *---------------------           ros_rt_wmp              --------------------
 *------------------------------------------------------------------------
 *                                                         V0.1B  15/09/11
 *
 *
 *  File: ./src/WMPServiceManager.h
 *  Authors: Danilo Tardioli
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

#ifndef WMPSERVICEMANAGER_H_
#define WMPSERVICEMANAGER_H_

#include "ServiceManager.h"

template<class T> class WMPServiceManager: public ServiceManager<T> {
public:
	WMPServiceManager(ros::NodeHandle * n, int port, std::string name) :
			ServiceManager<T>(n, port, name, 0) {
		n->deleteParam(ServiceManager<T>::param_dest);
	}

protected:

	void fillDestination(typename T::Request &req) {
		if (req.dest == wmpGetNodeId()) {
			ROS_ERROR(
					"Forbidden to call local service in a remote fashion, call will fail");
		} else {
			ServiceManager<T>::host = req.dest;
		}
	}

	int getPriority(typename T::Request &req) {
		return req.priority;
	}

};


#endif /* WMPSERVICEMANAGER_H_ */
