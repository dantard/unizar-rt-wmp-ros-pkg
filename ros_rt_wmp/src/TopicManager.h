/*------------------------------------------------------------------------
 *---------------------           ros_rt_wmp              --------------------
 *------------------------------------------------------------------------
 *                                                         V0.1B  15/09/11
 *
 *
 *  File: ./src/TopicManager.h
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

#ifndef TOPICMANAGER_H_
#define TOPICMANAGER_H_

#pragma GCC diagnostic ignored "-Warray-bounds"

#include "Manager.h"
	#include <zlib.h>

template<class T> class TopicManager: public Manager {
protected:
	std::map<std::string, info_t> flows_map;
	ros::Subscriber sub;
    int counter;
	ros::Publisher loop_publisher;
	unsigned int queue_size, period;
	T * emergency;
	bool isQoS, isWatchdog, topic_initied;
public:

	TopicManager(ros::NodeHandle * n, int port, std::string name,
			std::string source, std::string destination, unsigned char priority, bool broadcast) :
		Manager(n, port, name, priority) {
		amIstatic = true;
		setBroadcast(broadcast);
		setSource(source);
		setDestination(destination);
		init();
	}
	TopicManager(ros::NodeHandle * n, int port, std::string name,
			std::string source, std::string destination, unsigned char priority, bool broadcast, int _queue_size, int _period) :
		Manager(n, port, name, priority) {
		amIstatic = true;
		setBroadcast(broadcast);
		setSource(source);
		setDestination(destination);
		init();
		queue_size = _queue_size;
		period = _period;
		isQoS = true;
	}

	TopicManager(ros::NodeHandle * n, int port, std::string name,
			std::string source, std::string destination, unsigned char priority, int _period, T * _emergency) :
		Manager(n, port, name, priority) {
		amIstatic = true;
		setSource(source);
		setDestination(destination);
		init();
		period = _period;
		emergency = _emergency;
		isWatchdog = true;
	}


	TopicManager(ros::NodeHandle * n, int port, std::string name, std::string source, unsigned char priority, bool broadcast) :
		Manager(n, port, name, priority) {
		amIstatic = false;
		setBroadcast(broadcast);
		setSource(source);
		init();
	}

    void setPeriod(int ms){
        this->period = ms;
    }

	std::vector<T> buffer;
	virtual void pub_loop() {
		bool initied = false;
		while (ros::ok()){
			if (buffer.size() < queue_size && !initied){
				usleep(5000);
				continue;
			}
			initied = true;

			if (buffer.size()>0){
				loop_publisher.publish(buffer.front());
				buffer.erase(buffer.begin());
			}

			while(buffer.size() > queue_size ){
				buffer.erase(buffer.begin());
			}
			usleep(period*1000);
		}
	}

    void now(struct timespec *ts) {
        clock_gettime(CLOCK_REALTIME, ts);
    }

    unsigned int timestamp_ms() {
        struct timespec ts;
        now(&ts);
        unsigned long long res = ((unsigned long long)(ts.tv_sec)) * 1000000 + ((unsigned long long)(ts.tv_nsec)) / 1000;
        return (unsigned int)(res/1000);
    }

    std::vector<unsigned int> ticks;
    int count = 0;

    bool is_time_to_push(){

        ticks.push_back(timestamp_ms());
        if (ticks.size() > 5){
            ticks.erase(ticks.begin());
        }

        unsigned int sum = 0;
        for (int i = 0; i< ticks.size()-1; i++){
            sum += (ticks[i+1] - ticks[i]);
        }

        int period = ticks.size() > 1 ? sum/(ticks.size()-1): 0;
        if (period == 0 || this->period == 0){
            return true;
        }

        int rate = int(nearbyint(double(this->period)/double(period)));
        int real_period = rate*period;
        double error = fabs(1.0-(double(real_period)/double(this->period)));
        if (error > 0.05){
          //  ROS_WARN_ONCE("Topic '%s' real period is %dms", topic.c_str(), real_period);
        }
        if (++count > rate-1){
            count = 0;
            return true;
        }
        return false;
    }


	virtual void startRX(){
		ROSWMP_DEBUG(stderr,"am I dest? %d", amIdst);
		if (amIdst) {
			ROSWMP_DEBUG(stderr,"Queue subscribed (%s) port : %d", name.c_str(), port);

			boost::thread(boost::bind(&Manager::run, this));
			if (isQoS > 0){
				boost::thread(boost::bind(&TopicManager::pub_loop, this));
			}
		}
	}

	virtual bool isHost() {
		return amIsrc;
	}


	void init() {

		if (amIsrc) {
			std::ostringstream s;
			if (dests.size()>0){
				s << n->getNamespace() << "/tx/" << name;
			}else{
				s << n->getNamespace() << "/in/" << name;
			}
            sub = n->subscribe(s.str(), 10, &TopicManager::callback, this); //2017 -> ,ros::TransportHints().udp());
			ROSWMP_DEBUG(stderr,"Callback subscribed (%s)\n", s.str().c_str());

			if (! amIstatic){
				init_param();
			}

			std::ostringstream s1;
			s1 << n->getNamespace() << "/" << name << "/decimation";
			decimation = s1.str();
			n->setParam(decimation, 1);
		}
		counter = 1;
		queue_size = 0;
		period = 0;
		isQoS = false;
		isWatchdog = false;
		topic_initied = false;
	}

	bool shouldDecimate(){
		int val;
		if (!n->getParamCached(decimation, val)){
			val = 1;
		}

		if (val <= 0){
			n->setParam(decimation, 0);
			return true;
		}

		if (counter >= val){
			counter = 1;
			return false;
		}else{
			counter ++ ;
			return true;
		}
	}

	virtual void fillDestination(const boost::shared_ptr<T const> & message) {
		std::string value;
		n->getParamCached(param_dest, value);
		ROSWMP_DEBUG(stderr,"Get param cached: %s\n",value.c_str());
		setDestination(value);
	}

	virtual int getPriority(const boost::shared_ptr<T const> & message) {
		return flow_prio;
	}

	virtual int getSubPort(T & pm) {
		return 0;
	}
	virtual std::string getSubTopic(T & pm) {
		return "x";
	}

	virtual void callback(const boost::shared_ptr<T const> & message) {

        if (!justone && (stopped || shouldDecimate())) {
			return;
		}

        if (!is_time_to_push()){
            fprintf(stderr,"Not time to push\n");
            return;
        }
        fprintf(stderr,"Time to push\n");

		justone = false;

		if (!amIstatic){
			fillDestination(message);
		}

		if (dests.size()>0) {
			int priority = getPriority(message);
			int n = serialize<T>((char*)(sbuff + sizeof(flow_t)), message);
			ROSWMP_DEBUG(stderr,"Serializing size %d\n",n);
			int bc_dest = computeBroadcastDest();

			ROSWMP_DEBUG(stderr, "Push BC Message, size %d dest %d name %s\n", n + sizeof(flow_t),bc_dest,name.c_str());

			wmpPushData(port, sbuff, n + sizeof(flow_t), bc_dest, priority);

//			int size = n;
//			char * p = sbuff + sizeof(flow_t);
//			Bytef * zd = (Bytef *) malloc(500000);
//			uLongf zlen = 500000;
//			int err = compress(zd, &zlen, (const Bytef*) p, (uLong) size);
//			if (err != Z_OK) {
//				fprintf(stderr,"UNABLE to compress, sending uncompressed");
//			} else {
//				//memcpy(p, (char*) zd, zlen);
//				size = zlen;
//				//data_pointer = (char*) zd;
//			}
//			fprintf(stderr, "Post compress :%d\n", size);

		}else{
			std::ostringstream hash;
			hash << n->getNamespace() << "/decimated/" << name;
			ROSWMP_DEBUG(stderr, "Received %s on Manager\n", hash.str().c_str());
			if (flows_map.find(hash.str()) == flows_map.end()) {
				flows_map[hash.str()].publisher = n->advertise<T> (hash.str(),10);
				flows_map[hash.str()].publisher.publish(message);
				sleep(1);
			}
			flows_map[hash.str()].publisher.publish(message);
			ROSWMP_DEBUG(stderr, "Published (port:%d)\n!", port);
		}
	}

	virtual bool popMessage(T & pm, unsigned int & size, unsigned char & src1, signed char & pri) {
		char * p;

		int idx;
		if (!isWatchdog || !topic_initied){
			idx = wmpPopData(port, &p, &size, &src1, &pri);
		}else{
			idx = wmpPopDataTimeout(port, &p, &size, &src1, &pri, period);
		}

		if (idx == -1 ){
			if (!isWatchdog){
				wmpPopDataDone(idx);
				return false;
			}else{
				wmpPopDataDone(idx);
				pm = *emergency;
				ROS_WARN("Whatchdog raised on topic %s\n", name.c_str());

				return true;
			}
		}

		size = size - sizeof(flow_t);
		if (!deserialize<T>(p + sizeof(flow_t), size, pm)) {
			ROS_ERROR("Deserialize error\n");
			wmpPopDataDone(idx);
			return false;
		}

		wmpPopDataDone(idx);
		return true;
	}

	virtual void reconnect(){
		std::string s = sub.getTopic();
		sub.shutdown();
		sub = n->subscribe(s, 10, &TopicManager::callback, this);
	}

	virtual void run() {
		T pm;

		while (ros::ok()) {

			signed char pri;
			unsigned int size;
			unsigned char src1;

			if (!popMessage(pm, size, src1, pri)) {
				continue;
			}

			int subPort = getSubPort(pm);
			std::string subTopic = getSubTopic(pm);

			ROSWMP_DEBUG(stderr, "Received %s on Manager src %d \n", name.c_str(),src1);

			std::ostringstream hash;
			hash << n->getNamespace() << "/rx/R" << (int) src1 << "/" << name; //XXX: << "/compressed";

			if (subTopic.compare("x") != 0) {
				hash << "/" << subTopic;
			}
			if (subPort > 0) {
				hash << "/" << subPort;
			}

			ROSWMP_DEBUG(stderr, "Received %s on Manager\n", hash.str().c_str());


			if (flows_map.find(hash.str()) == flows_map.end()) {
				loop_publisher = flows_map[hash.str()].publisher = n->advertise<T> (hash.str(),10);
				flows_map[hash.str()].publisher.publish(pm);
				sleep(1);
			}

			if (isQoS > 0){
				buffer.push_back(pm);
			}else{
				flows_map[hash.str()].publisher.publish(pm);
				topic_initied = true;
			}
			///XXX: fprintf(stdout,"Publishing: %s                         \r",name.c_str());
			fflush(stdout);

			ROSWMP_DEBUG(stderr, "Published (port:%d)\n!", port);
		}
	}
};

#endif /* TOPICMANAGER_H_ */
