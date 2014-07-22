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
static int nnodes, node_id, write_messages;
static char filename[256];
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
#define WDW 50
#define L99P 60
int once = 1, firstone, gdiff = 0, idx = 0;
char inst[WDW];

void message_callback(wmpFrame * q) {
	int i = 0;
	static ros_rt_wmp_msgs::WMPMessageInfo v;
	char lqm[nnodes * nnodes];

	v.rx_rssi = q->hdr.rssi;

	//XXX:TMP
	if (once) {
		firstone = q->msg.cnt - 1;
		once = 0;
	}
	int diff = 0;
	static int last = q->msg.cnt - 1;
	diff = q->msg.cnt - last - 1;
	last = q->msg.cnt;
	gdiff += diff;

	inst[idx] = 0;
	idx++;
	idx = idx < WDW ? idx : 0;
	for (i = 0; i < diff; i++) {
		inst[idx] = 1;
		idx++;
		idx = idx < WDW ? idx : 0;
	}
	int sum = 0;
	for (i = 0; i < WDW; i++){
		sum += inst[i];
	}
	double measured_ma = 100.0 * double(sum) / double(WDW);
	double measured_global = 100.0 * double(gdiff) / double(last - firstone);

	v.serial = q->msg.cnt; // q->hdr.serial;
	v.loop_id = q->hdr.loop_id;
	v.header.stamp = ros::Time::now();
	v.src = q->msg.src;
	v.from = q->hdr.from;
	v.dest = q->msg.dest;
	v.message_part_id = q->msg.part_id;
	v.age = measured_ma + measured_global * 10000; //q->msg.age;
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
	for (i = 0; i < nnodes * nnodes; i++) {
		v.lqm.push_back(lqm[i]);
	}

	double expected = 1;
	for (i = 0; i < nnodes - 1; i++) {
		char i2 = q->msg.path[i];
		char i1 = q->msg.path[i + 1];
		if (i1 < 0) {
			continue;
		}
		fprintf(stderr, "i1:%d i2:%d val:%d\n", i1, i2, lqm[i1 * nnodes + i2]);
		expected = expected * ((double) lqm[i1 * nnodes + i2]) / 100.0;
	}

	fprintf(stderr, "VAL:%f\n", 100.0 - 100.0 * expected);

	int nhop = 0;
	double rssi = 1;
	for (i = 1; i < nnodes; i++) {
		int val=q->msg.rssi[i];
		if (q->msg.rssi[i] != 0){
			nhop++;
		}else{
			continue;
		}
		rssi = rssi * (double(val)/100.0);
		fprintf(stderr, "RSSI (%d):%d\n",i,val);
	}
	fprintf(stderr, "RSSI:%f\n", 100.0 - 100.0 * rssi);

	message_publisher.publish(v);

	if (write_messages) {
		/* write messages */
		static FILE * p = 0;
		if (p == 0) {
			if (strcmp(filename, "none")==0) {
				char time[256];
				getTimedFilename(time);
				sprintf(filename, "ros-rt-wmp-messages-n%d-%s.dat", node_id,time);
			}
			p = fopen(filename, "w+");
		}

		fprintf(p,
				"%llu %3d %3d %3d %3d %4d %4d %4d %4d %3d %5d %3d %3.1f %3.1f %3.1f %3.1f %3d    ",
				getRawActualTimeus(), q->msg.port, q->msg.src, q->msg.priority,
				q->msg.part_id, q->hdr.from, q->hdr.rssi, q->msg.len,
				q->msg.age, q->hdr.gretries, q->msg.cnt, (int) diff,
				(100.0 - 100.0 * expected),  measured_ma,
				measured_global, (100.0 - 100.0 * rssi), nhop);

		for (i = 0; i < nnodes; i++) {
			fprintf(p, "%2d ", q->msg.path[i]);
		}

		fprintf(p, "    ");

		for (i = 0; i < nnodes * nnodes; i++) {
			fprintf(p, "%3d ", lqm[i]);
		}

		fprintf(p, "    ");

		for (i = 1; i < nnodes; i++) {
			fprintf(p, "%3d ", q->msg.rssi[i]);
		}
		fprintf(p, "\n");
		fflush(p);
		/* write messages */
	}
}

void * publish_info(void *ptr) {
	while (ros::ok()) {
		ros_rt_wmp_msgs::WMPInfo pm;
		pm.serial = wmpGetSerial();
		pm.loop_id = wmpGetLoopId();
		pm.connected = wmpIsNetworkConnected();

		for (int i = 0; i < wmpGetNumOfNodes(); i++) {
			for (int j = 0; j < wmpGetNumOfNodes(); j++) {
				pm.lqm.push_back(lqm_get_val(i, j));
			}
		}

		int size = wmpGetNumOfNodes() * wmpGetNumOfNodes();
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
	char lqm[nnodes * nnodes];
	for (int i = 0; i < nnodes * nnodes; i++) {
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

	argo_setCommentId(argo_addInt(&write_messages, STR("write-messages"), 0, 0),
			STR("Write message log"));
	argo_setCommentId(argo_addString(filename, STR("filename"), STR("none")),
				STR("Log file name"));
	if (!wmpIsKernelSpace()) {
		argo_setCommentId(argo_addIntMandatory(&node_id, STR("node-id"), 0, 1),
				STR("Specify node WMP address"));
		argo_setCommentId(argo_addIntMandatory(&nnodes, STR("num-nodes"), 2, 1),
				STR("Specify number of nodes"));
		argo_setExample(argv[0],
				STR("--node-id 0 --num-nodes 3 --auto-namespace"));
	} else {
		argo_setExample(argv[0], STR("--auto-namespace"));
	}
	argo_doProcess(argc, argv, 0);

	wmpSetup(node_id, nnodes);

	if (strcmp(name, "") == 0) {
		snprintf(name, 31, "R%d", wmpGetNodeId());
	}

	ros::init(argc, argv, name);

	if (ans) {
		snprintf(ns, 31, "R%d", wmpGetNodeId());
	}

	ros::NodeHandle n(ns);

	message_publisher = n.advertise<ros_rt_wmp_msgs::WMPMessageInfo>("messages",
			1, true);
	WMPNodeManager w(&n, 2, std::string("wmp_control"));

	define_objects(&n, &w);

	wmpRunBG();
	if (wmpIsKernelSpace()) {
		printf("Node %s with id %d of %d is running.\n", name, wmpGetNodeId(),
				wmpGetNumOfNodes());
	}

	/* Message Callback */
	wmpSetMessageCallback(message_callback);

	/* Publish Info */
	pthread_t th;
	std::ostringstream oss;
	oss << n.getNamespace() << "/info";
	publisher = n.advertise<ros_rt_wmp_msgs::WMPInfo>(oss.str(), 1);
	pthread_create(&th, 0, publish_info, 0);

	/* Read Fake LQM */
	ros::Subscriber fklqm = n.subscribe("/fake_lqm", 1, fake_lqm_callback);

	ros::spin();
	return 0;

}

