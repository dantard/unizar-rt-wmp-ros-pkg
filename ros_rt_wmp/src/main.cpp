/*------------------------------------------------------------------------
 *---------------------           ros_rt_wmp              --------------------
 *------------------------------------------------------------------------
 *                                                         V0.1B  15/09/11
 *
 *
 *  File: ./src/main.cpp
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

#include <ros/ros.h>
#include <sstream>
#include <boost/thread.hpp>
#include <vector>
#include <ostream>
#include "config.h"
#include "Manager.h"
#include "TopicManager.h"
#include "ServiceManager.h"
#include "TFManager.h"
#include "WMPNodeManager.h"
#include "argon.h"
#include <ros_rt_wmp_msgs/WMPMessageInfo.h>

extern "C" {
#include "core/include/frames.h"

}
#define FRAME_INFO

static ros::Publisher message_publisher;
static ros::Publisher publisher;

void received(int *rtnCode, wmpFrame *p, wmpFrame*q) {
	static ros_rt_wmp_msgs::WMPFrames v;
	v.rssi = q->hdr.rssi;
	v.noise = q->hdr.noise;
	v.serial = q->hdr.serial;
	v.loop_id = q->hdr.loop_id;
	v.header.stamp = ros::Time::now();
	v.source = q->hdr.from;
	publisher.publish(v);
}

void message_callback(wmpFrame * q){
	static ros_rt_wmp_msgs::WMPMessageInfo v;
	v.rx_rssi = q->hdr.rssi;
	v.serial = q->hdr.serial;
	v.loop_id = q->hdr.loop_id;
	v.header.stamp = ros::Time::now();
	v.src = q->hdr.from;
	v.dest = q->msg.dest;
	v.message_part_id  = q->msg.part_id;
	v.age = q->msg.age;
	v.prio = q->msg.priority;
	v.size = q->msg.len;
	message_publisher.publish(v);
}

int main(int argc, char** argv) {
	char ns[32], name[32];
	int nnodes, node_id, ans;
	argo_setCommentId(argo_addString(name, STR("node-name"), STR("")),
			STR("Specify the name of the node"));
	argo_setCommentId(argo_addString(ns, STR("namespace"), STR("")),
			STR("Specify the namespace"));
	argo_setCommentId(argo_addInt(&ans, STR("auto-namespace"), 0, 0),
			STR("Obtain the namespace from the RT-WMP node_id"));

	if (!wmpIsKernelSpace()){
		argo_setCommentId(argo_addIntMandatory(&node_id, STR("node-id"), 0, 1),
				STR("Specify node WMP address"));
		argo_setCommentId(argo_addIntMandatory(&nnodes, STR("num-nodes"), 2, 1),
				STR("Specify number of nodes"));
		argo_setExample(argv[0],STR("--node-id 0 --num-nodes 3 --auto-namespace"));
	} else {
		argo_setExample(argv[0],STR("--auto-namespace"));
	}
	argo_doProcess(argc, argv, 0);

	wmpSetup(node_id, nnodes);

	if (strcmp(name,"")==0){
		snprintf(name,31,"R%d",wmpGetNodeId());
	}

	ros::init(argc, argv, name);

	if (ans){
		snprintf(ns,31,"R%d",wmpGetNodeId());
	}

	ros::NodeHandle n(ns);

	message_publisher = n.advertise<ros_rt_wmp_msgs::WMPMessageInfo>("messages",1,true);
	WMPNodeManager w(&n, 2, std::string("wmp_control"));

	define_objects(&n,&w);

	wmpRunBG();
	if (wmpIsKernelSpace()){
		printf("Node %s with id %d of %d is running.\n",name, wmpGetNodeId(), wmpGetNumOfNodes());
	}
	wmpSetMessageCallback(message_callback);

	ros::spin();
	return 0;

}

