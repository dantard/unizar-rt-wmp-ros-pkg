/*------------------------------------------------------------------------
 *---------------------           ros_rt_wmp              --------------------
 *------------------------------------------------------------------------
 *                                                         V0.1B  15/09/11
 *
 *
 *  File: ./src/WMPNodeManager.h
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

#ifndef WMPNODEMANAGER_H_
#define WMPNODEMANAGER_H_
extern "C"{
	char lqm_get_val(int i, int j);
}


class WMPNodeManager: public WMPServiceManager<ros_rt_wmp_msgs::WMPControl> {
	ros::NodeHandle *nh;
	std::map<std::string, Manager *> stuffs;
	ros::ServiceServer service;
public:

	WMPNodeManager(ros::NodeHandle * n, int port, std::string name) :

			WMPServiceManager<ros_rt_wmp_msgs::WMPControl>(n, port, name) {
		nh = this->n;
		service = n->advertiseService("wmp_control", &WMPNodeManager::manage,
				this);
		boost::thread(boost::bind(&WMPNodeManager::publishInfo, this));
		createThreads();
	}

	virtual ~WMPNodeManager() {

	}

	void publishInfo() {

		std::ostringstream oss;
		oss << nh->getNamespace() << "/info";
		ros::Publisher publisher = nh->advertise<ros_rt_wmp_msgs::WMPInfo>(oss.str(), 1);

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
	}

	void addManager(std::string p, Manager * v) {
		stuffs[p] = v;
		v->startRX();
	}

	bool manage(ros_rt_wmp_msgs::WMPControl::Request &req,
			ros_rt_wmp_msgs::WMPControl::Response &res) {
		if (req.dest != wmpGetNodeId()) {
			ROS_ERROR("This service is local only, returning local value");
		}
		ROSWMP_DEBUG(stderr, "Local service called with command %d, %s\n",req.command, req.param3.c_str());
		//fprintf (stderr, "Local service called with command %d, %s node %d \n",req.command, req.param3.c_str(), wmpGetNodeId());

		if (stuffs.find(req.param3) == stuffs.end() && req.command != STOP) {
			res.result = -1;
			res.info = "ERROR: Inexistent topic/service";
			return true;
		}
		if (req.dest != wmpGetNodeId()) {
			res.result = -1;
			res.info = "ERROR: Wrong destination parameter";
			return true;
		}
		Manager * m = stuffs[req.param3];
		if (m==NULL || !m->isHost()){
			res.result = -1;
			res.info = "ERROR: The specified node is not source/host of this topic/service";
			return true;
		}
		if (req.command == STOP) {
			res.info = "OK: COMMAND_LIST > 0:COMMAND_LIST, 1:TOPIC_STOP, 2:TOPIC_START, 3:TOPIC_DECIMATE, 4:TOPIC_JUSTONE, 5:TOPIC_SET_PRIORITY, 6:TOPIC_GET_PRIORITY, 7:RECONNECT";
		}else if (req.command == TOPIC_STOP) {
			res.info = "OK:TOPIC_STOP";
			m->stop();
		} else if (req.command == TOPIC_START) {
			res.info = "OK:TOPIC_START";
			m->start();
		}else if (req.command == TOPIC_DECIMATE) {
			m->setDecimation(req.param1);
			res.result = req.param1;
			res.info = "OK:TOPIC_DECIMATE";
		}else if (req.command == TOPIC_JUSTONE) {
			m->justOne();
			res.info = "OK:TOPIC_JUSTONE";
		}else if (req.command == SET_PRIORITY) {
			m->setPriority(req.param1);
			res.result = req.param1;
			res.info = "OK:TOPIC_SET_PRIORITY";
		}else if (req.command == GET_PRIORITY) {
			res.result = m->getPriority();
			res.info = "OK:TOPIC_GET_PRIORITY";
		}else if (req.command == RECONNECT) {
			m->reconnect();
			res.result = 0;
			res.info = "OK:RECONNECT";
		}
		else{
			res.info = "KO:UNKNOWN COMMAND";
		}
		return true;
	}
};


#endif /* WMPNODEMANAGER_H_ */
