/*------------------------------------------------------------------------
 *---------------------           ros_rt_wmp              --------------------
 *------------------------------------------------------------------------
 *                                                         V0.1B  15/09/11
 *
 *
 *  File: ./src/ServiceManager.h
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
#ifndef SERVICEMANAGER_H_
#define SERVICEMANAGER_H_

#include "Manager.h"

template<class T> class ServiceManager: public Manager {
protected:
	ros::Subscriber sub;
	ros::ServiceServer service;
	sem_t sem_resp;
	typename T::Response answ;
	int oks;
	int host;
public:
	ServiceManager(ros::NodeHandle * n, int port, std::string name, int host, unsigned char priority) :
		Manager(n, port, name, priority) {
		std::ostringstream oss;
		if (host != wmpGetNodeId()) {
			oss << n->getNamespace() << "/R" << host << "/" << name;
			service = n->advertiseService(oss.str(), &ServiceManager::callback, this);
		}
		this->host = host;
		amIstatic = true;
	};

	void startRX(){
		ROSWMP_DEBUG(stderr,"Starting RX threads\n");

		createThreads();
	}

	ServiceManager(ros::NodeHandle * n, int port, std::string name, unsigned char priority) :
		Manager(n, port, name, priority) {
		this->host = -1;
		amIstatic = false;
		std::ostringstream oss;
		oss << n->getNamespace() << "/remote/" << name;
		service = n->advertiseService(oss.str(), &ServiceManager::callback, this);
		init_param();
	}

	virtual void init_param() {
		std::ostringstream s1,s2;
		s1 << n->getNamespace() << "/" << name << "/dest";
		param_dest = s1.str();
		n->setParam(param_dest, host);
	}


	void createThreads() {
		sem_init(&sem_resp, 0, 0);
		boost::thread(boost::bind(&ServiceManager::run, this));
		boost::thread(boost::bind(&ServiceManager::waitNetAnswer, this));
	}


	virtual void fillDestination(typename T::Request &req) {
		int val;
		if (!amIstatic) {
			if (n->getParamCached(param_dest, val)){
				host = val;
			}
		}
	}

	virtual int getPriority(typename T::Request &req) {
		return flow_prio;
	}

	bool callback(typename T::Request &req, typename T::Response &resp) {
		ROSWMP_DEBUG(stderr, "Callback received");

		int n = serialize<typename T::Request> ((sbuff + sizeof(flow_t)), req);

		if (!amIstatic){
			fillDestination(req);
		}
		int priority = getPriority(req);
		ROSWMP_DEBUG(stderr, "Sending request with priority: %d and host: %d", priority,host);
		if (host >= 0){
			wmpPushData(port, sbuff, n + sizeof(flow_t), 1 << host, priority);
		} else{
			ROS_ERROR("No destination specified, set .../dest param");
			return false;
		}

		timespec ts;
		clock_gettime(CLOCK_REALTIME, &ts);
		ts.tv_sec += 3;
		oks = 1;
		if (sem_timedwait(&sem_resp, &ts) == 0 && oks) {
			ROSWMP_DEBUG(stderr,"Received port = %d",port);
			resp = answ;
			return true;
		} else {
			return false;
		}
	}


	void run() {
		typename T::Request req;
		ros::ServiceClient client = n->serviceClient<T> (name);
		while (ros::ok()) {
			char * p;
			signed char pri;
			unsigned int size;
			unsigned char src;
			ROSWMP_DEBUG(stderr, "Popping (wait) on port %d",port);
			int idx = wmpPopData(port, &p, &size, &src, &pri);
			flow_t * f = (flow_t *) p;

			ROSWMP_DEBUG(stderr, "Deserializing");
			deserialize<typename T::Request>((p + sizeof(flow_t)),size - sizeof(flow_t), req);
			ROSWMP_DEBUG(stderr, "Deserialized");

			wmpPopDataDone(idx);

			/* Here I have the request */
			T srv;
			srv.request = req;

			f = (flow_t *) sbuff;

			ROSWMP_DEBUG(stderr, "Call service: %s",name.c_str());

			if (client.call(srv)) {
				ROSWMP_DEBUG(stderr, "Call ok");
				f->ok = 1;
			} else {
				ROSWMP_DEBUG(stderr, "Call error");
				f->ok = 0;
			}

			ROSWMP_DEBUG(stderr, "Post call");
			//uint8_t unused = 0;

			int n = serialize<typename T::Response>((sbuff + sizeof(flow_t)),srv.response);
			//int n = srv.response.serializationLength();
			//srv.response.serialize((uint8_t *) (buff + sizeof(flow_t)), unused);

			wmpPushData(port + 1, sbuff, n + sizeof(flow_t), 1 << src, getPriority(req));
			ROSWMP_DEBUG(stderr, "Pushed response");
		}
	}

	void waitNetAnswer() {
		while (ros::ok()) {
			char * p;
			signed char pri;
			unsigned int size;
			unsigned char src;
			int idx = wmpPopData(port + 1, &p, &size, &src, &pri);
			ROSWMP_DEBUG(stderr, "Received on Manager\n");
			flow_t * f = (flow_t *) p;

			//answ.deserialize((uint8_t*) (p + sizeof(flow_t)));

			deserialize<typename T::Response>((p + sizeof(flow_t)), size - sizeof(flow_t), answ);

			ROSWMP_DEBUG(stderr, "Deserializing Answer\n");
			oks = f->ok;
			wmpPopDataDone(idx);
			ROSWMP_DEBUG(stderr, "Deserialized Answer result = %d  \n", oks);
			sem_post(&sem_resp);
		}
	}
};


#endif /* SERVICEMANAGER_H_ */
