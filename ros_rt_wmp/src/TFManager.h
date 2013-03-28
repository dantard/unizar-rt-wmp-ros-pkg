/*------------------------------------------------------------------------
 *---------------------           ros_rt_wmp              --------------------
 *------------------------------------------------------------------------
 *                                                         V0.1B  15/09/11
 *
 *
 *  File: ./src/TFManager.h
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


#ifndef TFMANAGER_H_
#define TFMANAGER_H_

#include <ros/ros.h>
#include <sstream>
#include <boost/thread.hpp>
#include <vector>
#include <ostream>
#include "config.h"
#include "Manager.h"
#include <tf/tfMessage.h>
#include <boost/shared_ptr.hpp>


class TFManager: public Manager {
	struct info_t{
		tf::tfMessage tfm;
		bool sent;
		ros::Time ts;
	};
	std::map<std::string, info_t> tf_map;
	ros::Subscriber sub;
	pthread_mutex_t mtx;
public:

	TFManager(ros::NodeHandle * n, int port, std::string source, std::string destination, unsigned char priority, bool broadcast) : Manager(n,port,"/tf", priority) {
		amIstatic = true;
		setBroadcast(broadcast);
		setDestination(destination);
		setSource(source);
	}

	TFManager(ros::NodeHandle * n, int port, std::string source,unsigned char priority, bool broadcast) : Manager(n,port,"/tf", priority) {
		amIdst = true;
		amIstatic = false;
		setBroadcast(broadcast);
		setSource(source);
		init_param();
	}
	virtual void fillDestination() {
		std::string val;
		n->getParamCached(param_dest, val);
		ROSWMP_DEBUG(stderr,"Get param cached: %s\n",val.c_str());
		setDestination(val);
	}

	virtual void startRX() {
		if (amIsrc){
			sub = n->subscribe(name, 10, &TFManager::callback, this);
			boost::thread(boost::bind(&TFManager::run, this));
			ROS_INFO("TF Tx subscribed (%s)",name.c_str());
		}
		if (amIdst){
			boost::thread(boost::bind(&TFManager::waitNetData, this));
		}
		pthread_mutex_init(&mtx,0);
	}

	void callback(const boost::shared_ptr<tf::tfMessage const> & message) {
		pthread_mutex_lock(&mtx);
		std::vector< geometry_msgs::TransformStamped > ros_vec;
//		message->get_transforms_vec (ros_vec);
		std::string tf_prefix;
		bool found = n->getParamCached("/tf_prefix",tf_prefix);

		if (!found){
			ROS_WARN("Please specify a tf_prefix present: %s",tf_prefix.c_str());
		}
		if (ros_vec.at(0).header.frame_id.find(tf_prefix)!=std::string::npos or ros_vec.at(0).child_frame_id.find(tf_prefix) != std::string::npos){
			std::string s = ros_vec.at(0).header.frame_id + "-" + ros_vec.at(0).child_frame_id;
			tf_map[s].tfm = *message;
			tf_map[s].sent = false;
			tf_map[s].ts = ros::Time::now();
		}else{
			ROSWMP_DEBUG(stderr,"Discarding foreign prefix %s", tf_prefix.c_str());
		}
		pthread_mutex_unlock(&mtx);
	}

	void waitNetData(){
		char * rxbuff;
		signed char priority;
		unsigned int size;
		unsigned char src;
		tf::tfMessage m;
		ros::Publisher publisher = n->advertise<tf::tfMessage> ("/tf",1);

		while (1){
			int offset = 0;
			ROSWMP_DEBUG(stderr,"Popping msg\n");
			int id = wmpPopData(port,&rxbuff,&size,&src,&priority);
			ROSWMP_DEBUG(stderr,"Popped msg of size: %d\n",size);
			flow_t * fw = (flow_t *) rxbuff;
			int npaks = fw->npaks;
			for (int i = 0; i < npaks ; i++){
				fw = (flow_t *) (rxbuff + offset);
				ROSWMP_DEBUG(stderr,"Publish %dth TF data fw->len = %d offset = %d size: %d\n", i, fw->len, offset, size);
				//m.deserialize((uint8_t*) (buff + offset + sizeof(flow_t)));
				if (!deserialize<tf::tfMessage>(rxbuff + offset + sizeof(flow_t),fw->len,m)){
					break;
				}
				for (unsigned i = 0; i< m.transforms.size(); i++){
					m.transforms.at(i).header.stamp = ros::Time::now();
				}
				publisher.publish(m);
				offset+= sizeof(flow_t) + fw->len;
			}
			wmpPopDataDone(id);
		}
	}
	void run(){
		while (1){
			pthread_mutex_lock(&mtx);
			if (tf_map.size() > 0) {

				if (!amIstatic){
					fillDestination();
				}
				if (dests.size() > 0) {

					int offset = 0, size = 0, npaks = 0;
					std::map<std::string, info_t>::iterator it;
					for (it = tf_map.begin(); it != tf_map.end(); it++) {
						if (!it->second.sent){
							flow_t * fw = (flow_t *) (sbuff + offset);
							npaks++;

							int len = serialize<tf::tfMessage> (
								(char*) (sbuff + sizeof(flow_t) + offset),
								it->second.tfm);
							it->second.sent = true;

							fw->len = len;
							offset += (len + sizeof(flow_t));
							size += (len + sizeof(flow_t));
						}else{
							ROSWMP_DEBUG(stderr,"Discarding already sent\n");
						}
					}
					flow_t * fw = (flow_t *) (sbuff);
					fw->npaks = npaks;

					if (size > 0) {
						int dest = computeBroadcastDest();
						wmpPushData(port, sbuff, size, dest, 99);
					}
				}
			}
			pthread_mutex_unlock(&mtx);
			usleep(20000);
		}
	}
};
#endif
