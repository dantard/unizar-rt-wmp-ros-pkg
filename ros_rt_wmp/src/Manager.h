/*------------------------------------------------------------------------
 *---------------------           ros_rt_wmp              --------------------
 *------------------------------------------------------------------------
 *                                                         V0.1B  15/09/11
 *
 *
 *  File: ./src/Manager.h
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
#ifndef MANAGER_H_
#define MANAGER_H_
#include <map>
#include <sstream>
#include <semaphore.h>
#include <sstream>
#include <vector>
#include <boost/algorithm/string.hpp>

extern "C" {
#include "core/interface/wmp_interface.h"
#include "core/interface/Msg.h"
#include "core/include/definitions.h"
}

#define MAX_DATA_SIZE	512000
#define WMP_MESSAGE_SIZE  1000
#define ROSWMP_DEBUG(output,...)   // fprintf(output, __VA_ARGS__); fprintf(output,"\n");

struct info_t {
	ros::Publisher publisher;
};


struct flow_t {
	int type;
	int ok;
	int len;
	int npaks;
};

class Manager {
protected:
	int port;
	unsigned char flow_prio;
	std::vector<uint8_t> dests;
	bool bc, stopped, amIsrc, amIdst,amIstatic;
	std::string name, param_dest, decimation;
	bool justone;
	char sbuff[MAX_DATA_SIZE];
	boost::shared_array<uint8_t> dbuff;

	enum wmp_commands_t {
		STOP, TOPIC_STOP, TOPIC_START, TOPIC_DECIMATE, TOPIC_JUSTONE, SET_PRIORITY, GET_PRIORITY
	};
public:
	ros::NodeHandle * n;
	Manager(ros::NodeHandle * n, int port, std::string name, unsigned char priority) {
		this->n = n;
		this->port = port;
		this->name = name;
		this->flow_prio = priority;
		amIdst = true;
		amIsrc = false;
		amIstatic = true;
		stopped = false;
		dbuff.reset(new unsigned char[MAX_DATA_SIZE]);
		justone = false;
	}

	void setBroadcast(bool bc){
		this->bc = bc;
	}

	void justOne(){
		justone = true;
	}

	void setSource(std::string source){
		std::vector<std::string> strs;
		boost::split(strs, source, boost::is_any_of(" ,n/"));
		for (unsigned i = 0; i < strs.size();i++){
			if (strs.at(i).compare("")!=0){ 
				if (atoi(strs.at(i).c_str()) == wmpGetNodeId()){
					amIsrc = true;
				}
			}
		}
	}

	void setDestination(std::string destination) {
		dests.clear();
		std::vector<std::string> strs;
		boost::split(strs, destination, boost::is_any_of(" ,n/"));
		for (unsigned i = 0; i < strs.size(); i++) {
			if (strs.at(i).compare("")!=0){
				int id = atoi(strs.at(i).c_str());
				if (id == wmpGetNodeId()) {
					amIdst = true;
					//dests.push_back(id);//XXX: AGGIUNTO 5 nov 2012
				} else {
					dests.push_back(id);
				}
				//std::cerr << id << " dest" << std::endl;
			}
		}
	}

	int computeBroadcastDest(){
		int dest = 0;
		for (unsigned i = 0; i < dests.size(); i++) {
			dest = dest + int(pow(2,dests[i]));
		}
		return dest;
	}

	virtual void init_param() {
	
		std::ostringstream s1,s2;
		s1 << n->getNamespace() << "/" << name << "/dest";
		param_dest = s1.str();

		for (unsigned i = 0; i < dests.size(); i++){
			if (i == 0){
				s2 << int(dests.at(i));
			} else {
				s2 << "," << int(dests.at(i));
			}
		} 
		//std::cerr<< "yes " << param_dest << " " << s2.str() << "\n";
		n->setParam(param_dest, s2.str());
	}

	virtual void startRX(){
	}

	virtual unsigned char getPriority(){
		return flow_prio;
	}

	virtual void setPriority(unsigned char prio){
		flow_prio = prio;
	}


	virtual void run()=0;

	virtual void stop() {
		ROSWMP_DEBUG(stderr, "Stopping %s\n",name.c_str());
		stopped = true;
	}
	
	virtual void start() {
		ROSWMP_DEBUG(stderr, "Starting %s\n",name.c_str());
		stopped = false;
	}

	virtual bool isHost() {
		return false;
	}

	template <typename Q> bool deserialize(char * p, int size, Q & pm){
		memcpy(dbuff.get(), p, size);
		ROSWMP_DEBUG(stderr, "DESERIALIZE SIZE: %d %s \n",size,name.c_str());
		try {
			ros::SerializedMessage smsg(dbuff, size);
			ros::serialization::deserializeMessage<Q>(smsg, pm);

			ros::Time* tm = ros::message_traits::timeStamp(pm);
			if (tm){
				ROSWMP_DEBUG(stderr,"Type %s, tm", ros::message_traits::datatype(pm));
				*tm = ros::Time::now();
			}else{
				ROSWMP_DEBUG(stderr,"Type %s, no TS", ros::message_traits::datatype(pm));
			}

			return true;
		} catch (...){
			ROS_ERROR("DESERIALIZE ERROR IN %s\n", name.c_str());
			return false;
		}
	}
	template <typename P> int serialize(char * p, const boost::shared_ptr<P const> & pm){
		ros::SerializedMessage sbuffer = ros::serialization::serializeMessage<P>(*pm);
		memcpy(p,sbuffer.message_start,sbuffer.num_bytes);
		return sbuffer.num_bytes;
	}

	template <typename P> int serialize(char * p, P & pm){
		ros::SerializedMessage sbuffer = ros::serialization::serializeMessage<P>(pm);
		memcpy(p,sbuffer.message_start,sbuffer.num_bytes);
		return sbuffer.num_bytes;
	}
	void setDecimation(int i){
		n->setParam(decimation, i);
	}
};

#endif /* MANAGER_H_ */
