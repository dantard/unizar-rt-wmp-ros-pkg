/*------------------------------------------------------------------------
 *---------------------           ros_rt_wmp              --------------------
 *------------------------------------------------------------------------
 *                                                         V0.1B  15/09/11
 *
 *
 *  File: ./src/WMPTopicManager.h
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

#ifndef WMPTOPICMANAGER_H_
#define WMPTOPICMANAGER_H_

#include "TopicManager.h"
#include <ros_rt_wmp_msgs/WMPControl.h>
#include <ros_rt_wmp_msgs/WMPInfo.h>
#include <ros_rt_wmp_msgs/WMPFrames.h>

template<class T> class WMPTopicManager: public TopicManager<T> {
public:
	WMPTopicManager(ros::NodeHandle * n, int port, std::string name,
			std::string src, bool broadcast) :
			TopicManager<T>(n, port, name, src, 0, broadcast) {
		n->deleteParam(TopicManager<T>::param_dest);
	}

	virtual void fillDestination(const boost::shared_ptr<T const> & message) {
		Manager::dests = message->header.dest;
	}

	virtual int getPriority(const boost::shared_ptr<T const> & message) {
		return message->header.priority;
	}
	std::string getSubTopic(T & pm) {
		return pm.header.sub_topic;
	}
};


#endif /* WMPMANAGER_H_ */
