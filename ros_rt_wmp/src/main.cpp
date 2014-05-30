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
 #include "core/include/wmp_misc.h"
}
#define FRAME_INFO

static ros::Publisher message_publisher;
static ros::Publisher publisher;
static int nnodes, node_id;

//void received(int *rtnCode, wmpFrame *p, wmpFrame*q) {
//	static ros_rt_wmp_msgs::WMPFrames v;
//	v.rssi = q->hdr.rssi;
//	v.noise = q->hdr.noise;
//	v.serial = q->hdr.serial;
//	v.loop_id = q->hdr.loop_id;
//	v.header.stamp = ros::Time::now();
//	v.source = q->hdr.from;
//	publisher.publish(v);
//}

void message_callback(wmpFrame * q){
	int i = 0;
	static ros_rt_wmp_msgs::WMPMessageInfo v;
	char lqm[nnodes*nnodes];

	v.rx_rssi = q->hdr.rssi;
	v.serial = q->hdr.serial;
	v.loop_id = q->hdr.loop_id;
	v.header.stamp = ros::Time::now();
	v.src = q->msg.src;
	v.from = q->hdr.from;
	v.dest = q->msg.dest;
	v.message_part_id  = q->msg.part_id;
	v.age = q->msg.age;
	v.prio = q->msg.priority;
	v.size = q->msg.len;
	v.ts = getRawActualTimeus();
	v.port = q->msg.port;

	v.path.clear();
	for (i = 0; i < nnodes; i++) {
		v.path.push_back(q->msg.path[i]);
	}

	v.lqm.clear();
	wmpGetLatestLQM(lqm);
	for (i = 0; i < nnodes*nnodes; i++) {
		v.lqm.push_back(lqm[i]);
	}

	message_publisher.publish(v);
}


void * publish_info( void *ptr ) {
	while (ros::ok()) {
		ros_rt_wmp_msgs::WMPInfo pm;
		pm.serial = wmpGetSerial();
		pm.loop_id = wmpGetLoopId();
		pm.connected = wmpIsNetworkConnected();

		for (int i = 0; i<wmpGetNumOfNodes(); i++){
			for (int j = 0; j<wmpGetNumOfNodes(); j++){
				pm.lqm.push_back(lqm_get_val(i,j));
			}
		}

		int size = wmpGetNumOfNodes()*wmpGetNumOfNodes();
		char distances[size];
		wmpGetLatestDistances(distances);
		for (int i = 0; i < size; i++) {
			pm.dist.push_back(distances[i]);
		}

		publisher.publish(pm);
		usleep(250000);
	}
	return NULL;
}

void fake_lqm_callback(const ros_rt_wmp_msgs::WMPInfo& msg) {
	char lqm[nnodes*nnodes];
	for (int i = 0; i < nnodes*nnodes; i++){
		lqm[i] = msg.lqm[i];
	}
	wmpForceLQM(lqm);
}


int main(int argc, char** argv) {
	char ns[32], name[32];
	int ans;
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

	/* Message Callback */
	wmpSetMessageCallback(message_callback);

	/* Publish Info */
	pthread_t th;
	std::ostringstream oss;
	oss << n.getNamespace() << "/info";
	publisher = n.advertise<ros_rt_wmp_msgs::WMPInfo>(oss.str(), 1);
	pthread_create(&th,0,publish_info, 0);

	/* Read Fake LQM */
	ros::Subscriber fklqm = n.subscribe("/fake_lqm", 1,fake_lqm_callback);

	ros::spin();
	return 0;

}

