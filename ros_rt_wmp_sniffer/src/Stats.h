/*------------------------------------------------------------------------
 *---------------------           WMPSNIFFER          --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: Stats.h
 *  Authors: Danilo Tardioli
 *  ----------------------------------------------------------------------
 *  Copyright (C) 2000-2012, Universidad de Zaragoza, SPAIN
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


#ifndef STATS_H1_
#define STATS_H1_
#include <stdlib.h>
#include <string>
#include <sstream>
#include <queue>
#include <limits>
#include "wmp_config.h"

#include "core/include/frames.h"
#include "core/interface/Msg.h"
#include <string.h>
#include "misc.h"

template<class T>
std::string toString(T val, int precision = -1) {
	std::ostringstream oss;
	oss << std::fixed;
	if (precision != -1) {
		oss.precision(precision);
	}
	oss << val;
	return oss.str();
}
;

template<class T>
class CStats {

protected:
	int id;
	bool id_valid;
	std::string name, unit;
	T val, sum;
	bool valid;
public:
	T bookmark, aux1, aux2, partial, total;
	long long llaux;
	struct stat_t {
		T value;
		int pos;
	};

	virtual void init(){
		valid=id_valid=false;
		sum=id=val=aux1=aux2=bookmark=llaux=partial=total=0;
	}

	CStats() {
		init();
		this->name="xxx";
		this->unit="yyy";
	}
	CStats(std::string & name) {
		this->name = name;
		init();
	}

	std::string & getName() {
		return name;
	}

	void set(T val) {
		this->val = val;
	}

	T get() {
		return this->val;
	}
	bool isValid() {
		return valid;
	}

	virtual bool new_value(){
		partial++;
		return true;
	}

	virtual bool new_touch(){
		total++;
		return true;
	}

	double getPercentage(){
		if (total!=0){
			return (100.0*(double) partial / (double) total);
		}else{
			return 0;
		}
	}

	std::string & getUnit() {
		return unit;
	}
};

template<class T>
class PlainCStats: public CStats<T> {
typedef	typename CStats<T>::stat_t stat_t;
public:
	virtual void init(){
		CStats<T>::init();
	}
	PlainCStats() : CStats<T>() {
		init();
	}
	PlainCStats(std::string name,std::string unit):CStats<T>(name) {
		this->unit = unit;
		init();
	}

	void new_value(T value) {
		CStats<T>::new_value();
		this->valid = true;
		this->sum+=value;
	}
	T getSum() {
		return this->sum;
	}

};

template<class T>
class MaxMinCStats: public CStats<T> {
typedef	typename CStats<T>::stat_t stat_t;
protected:
	double mean;
	stat_t max, min;
	int idx;
	std::vector<T> v;
	std::vector<T> vpos;
public :
	virtual void init() {
			CStats<T>::init();
			max.value = std::numeric_limits<T>::min();
			min.value = std::numeric_limits<T>::max();
			max.pos = -1;
			min.pos = -1;
			idx = 0;
			mean = 0;
			v.clear();
			vpos.clear();
	}

	void reset(){
		max.value = std::numeric_limits<T>::min();
		min.value = std::numeric_limits<T>::max();
		max.pos = -1;
		min.pos = -1;
		idx = 0;
		mean = 0;
	}

	MaxMinCStats():CStats<T>() {
		init();
	}
	MaxMinCStats(std::string name, std::string unit):CStats<T>(name) {
		this->unit = unit;
		init();
	}
	void new_value(T value, int pos) {
		v.push_back(value);
		vpos.push_back(pos);
		this->valid = true;
		if (value > max.value) {
			max.value=value;
			max.pos = pos;
		}
		if (value < min.value) {
			min.value = value;
			min.pos = pos;
		}
		idx++;
		this->sum += value;
	}
	int getCount() {
		return idx;
	}

	T get(int i){
		return v.at(i);
	}
	T getPos(int i){
		return vpos.at(i);
	}
	stat_t getMax() {
		return max;
	}
	T getMaxVal(){
		return max.value;
	}

	T getMinVal(){
		return min.value;
	}

	T getInterval() {
		return (max.value - min.value);
	}
	stat_t getMin() {
		return min;
	}
	virtual double getMean() {
		if (idx!=0){
			return (double) this->sum / (double) idx;
		} else{
			return 0;
		}
	}
};

template<class T>
class IntervalCStats: public MaxMinCStats<T> {
typedef	typename CStats<T>::stat_t stat_t;
	stat_t best_max;
	stat_t best_min;
	int best_count;
public:
	virtual void init(){
		MaxMinCStats<T>::init();
		best_count=0;
	}

	IntervalCStats():MaxMinCStats<T>() {
		init();
	}
	IntervalCStats(std::string name, std::string unit):MaxMinCStats<T>(name, unit) {
		init();
	}
	void reset() {
		done();
		MaxMinCStats<T>::reset();
	}
	void done(){
		if (this->idx > best_count) {
			best_max = this->max;
			best_min = this->min;
			best_count = this->idx;
		}
	}

	stat_t getIntervalMax() {
		return best_max;
	}
	stat_t getIntervalMin() {
		return best_min;
	}
	int getIntervalCount() {
		return best_count;
	}
};

template<class T, class P>
class ThingOverTimeCStats: public MaxMinCStats<T> {
typedef	typename CStats<T>::stat_t stat_t;
	P things;
public:
	virtual void init(){
		MaxMinCStats<T>::init();
		things = 0;
	}
	ThingOverTimeCStats():MaxMinCStats<T>() {
		this->init();
	}
	ThingOverTimeCStats(std::string name, std::string unit):MaxMinCStats<T>(name, unit) {
		this->init();
	}
	void new_thing(T size) {
		this->things+=size;
	}
	void new_time(T time) {
		this->new_value(time,0);
	}
	double getToT() {
		if ((this->max.value - this->min.value) != 0){
			return (double)(((double) things) / ((double) (this->max.value - this->min.value)));
		} else{
			return 0;
		}
	}
	virtual double getMean() {
		return getToT();
	}
};

#include <vector>

template<class T>
class ListCStats: public CStats<T> {
typedef	typename CStats<T>::stat_t stat_t;
protected:
	std::vector<stat_t> v;
	std::string child_name;
public:
	virtual void init(){
		CStats<T>::init();
		v.clear();
	}
	ListCStats(std::string name, std::string child_name) : CStats<T>(name) {
		this->child_name = child_name;
		init();
	}
	ListCStats() : CStats<T>() {
		init();
	}
	void new_value(T value, int pos) {
		CStats<T>::new_value();
		this->valid = true;
		stat_t elem;
		elem.value=value;
		elem.pos=pos;
		v.push_back(elem);
	}

	virtual int getCount() {
		return v.size();
	}

	virtual stat_t getElem(int idx) {
		stat_t elem = v.at(idx);
		return elem;
	}

	std::string & getChildName() {
		return child_name;
	}

};
template<class T>
class DoubleListCStats: public ListCStats<T> {
typedef	typename CStats<T>::stat_t stat_t;

public:
	DoubleListCStats(std::string name, std::string child_name) : ListCStats<T>(name,child_name) {
	}
	DoubleListCStats() : ListCStats<T>() {
	}

	void new_value(T value1, int pos1, T value2, int pos2) {
		ListCStats<T>::new_value(value1,pos1);
		ListCStats<T>::new_value(value2,pos2);
	}
	void getElem(int pos, stat_t & v1, stat_t & v2) {
		v1 = ListCStats<T>::getElem(pos*2);
		v2 = ListCStats<T>::getElem(pos*2+1);
	}
	virtual int getCount() {
		return ListCStats<T>::getCount()/2;
	}
	stat_t getElem(int idx) {
		stat_t elem = ListCStats<T>::getElem(idx*2);
		return elem;
	}
};

template<class Q>
class MultiCStats {
protected:
	std::string name, child_name;
	std::vector<Q> v2;
public:
	MultiCStats(std::string name, std::string child_name, int nfields) {
		this->child_name = child_name;
		this->name = name;
		v2.resize(nfields);
	}
	Q & operator[](int id) {
		return v2.at(id);
	}

	Q & get(int id) {
		return v2.at(id);
	}
	std::string & getChildName() {
		return child_name;
	}
	std::string & getName() {
		return name;
	}
};

#include <map>

class StatsFlow {
	struct Interval {
		int begin;
		int end;
		int dest;
		int rescheduled;
		int prio;
		int age;
		long long timeBegin, timeEnd, timeLoopBegin;
		wmpFrame p;
		struct {
			unsigned short first;
			int firstPos;
			unsigned short last;
			int lastPos;
		} LoopId;
	};
	std::map<unsigned int, Interval> messages;
	std::map<unsigned int, Interval> bc_messages;
	std::map<int, MaxMinCStats<long long> * > prio;
	std::map<int, MaxMinCStats<long long> * >::iterator prioIt;
	MaxMinCStats<long long> mdd;
	MaxMinCStats<long long> papMdd;
	MaxMinCStats<long long> interAuth;
	MaxMinCStats<long long> ctt;
	MaxMinCStats<long long> cta;
	MaxMinCStats<long long> ctm;


	ListCStats<long long> interAuthList;
	ListCStats<long long> nrd;
	ThingOverTimeCStats<long long, long long> bw;
	PlainCStats<int> nmsg;
	DoubleListCStats<int> preempted;
	ListCStats<int> rescheduled;
	ListCStats<int> orphan;
	DoubleListCStats<int> o3;
	ThingOverTimeCStats<long long, long long> avail;




	bool initied, isqos;
	int id, from, to, src, dst , begin_id, loop_id;

	struct {
		int Pos, NTPos, MKey, msg_id_pos;
		long long NTTime, Time;
		unsigned int msg_id;
		wmpFrame Frame;
	} prev;

public:
	virtual void init(){
		messages.clear();
		mdd.init();

		ctt.init();
		cta.init();
		ctm.init();

		interAuth.init();
		interAuthList.init();
		nrd.init();
		bw.init();
		nmsg.init();
		preempted.init();
		rescheduled.init();
		papMdd.init();
		avail.init();
		initied=false;
		begin_id=0;
		loop_id=0;
		memset(&prev,0,sizeof(prev));

		prev.NTTime = 0;
		prev.NTPos = 0;
	}

	StatsFlow(int from, int to, bool qos, long long time_begin = 0) :
			mdd("Message Delivery Delay", "us"), papMdd("Message Delivery Delay + PAP", "us"), nrd(
					"Message Not Reaching Destination", "us"), bw("Bandwidth",
					"Kbps"), nmsg("Messages", ""), preempted("Preempted","Msg Univocal Id"), rescheduled("Rescheduled","Id"),orphan("Orphan","Id"),
					o3("Out Of Order","Id"), avail("Available time per second","ms"), ctt("Tokens","us"), cta("Auths","us"), ctm("Messages","us"){
		this->src=from;
		this->dst=to;
		this->isqos=qos;
		init();
		bw.new_time(0);
	}
	int get_src(){
		return src;
	}
	int get_dst(){
		return dst;
	}
	bool is_qos(){
		return isqos;
	}
	void new_frame(wmpFrame * p, long long time, int pos){
		bw.new_time(time);
		avail.new_time(time);
		if (p->hdr.type == TOKEN){
			if (p->tkn.beginner == p->hdr.from){
				prev.NTTime = prev.Time;
				prev.NTPos = prev.Pos;
				loop_id = p->hdr.loop_id;
			}
		}
		prev.Pos = pos;
		prev.Time = time;
		prev.Frame = *p;
		if (p->hdr.type == MESSAGE){
			new_message(p, time, pos);
		}


	}

	int begin(){
		return this->begin_id;
	}

	void new_message(wmpFrame * p, long long time, int pos) {
//		bool is_last_msg_jump = (p->msg.dest & (1 << p->hdr.to)) && ((dst < 0) || (p->msg.dest & 1 << dst));
		bool is_last_msg_jump = (p->msg.dest & (1 << p->hdr.to)) && ((dst < 0) || (p->hdr.to == dst));

		bool is_first_msg_jump = ((p->msg.src == p->hdr.from) && ((dst < 0) || (p->msg.dest | 1 << dst)));

		unsigned int mkey = p->msg.msg_hash*100000 + 1000*abs(p->msg.part_id) + 100*p->msg.port;

		if (!initied){
			begin_id = pos;
			prev.Pos = pos;
			prev.Time = time;
			initied = true;
		}
		if (messages.find(mkey) == messages.end()){
			messages[mkey].begin = 0;
			messages[mkey].end = 0;
			messages[mkey].prio = p->msg.priority;
			messages[mkey].age = p->msg.age;
			nmsg.new_value(1);
		}

		if (is_first_msg_jump) {

			messages[mkey].begin = pos;
			messages[mkey].timeBegin = prev.Time;
			messages[mkey].dest = p->msg.dest;

			messages[mkey].rescheduled = mBitsIsSet(p->msg.type,2);

			messages[mkey].LoopId.first = p->hdr.loop_id;
			messages[mkey].LoopId.firstPos = pos;

			if (p->hdr.loop_id == loop_id){
				messages[mkey].timeLoopBegin = prev.NTTime;
			}else{
				messages[mkey].timeLoopBegin = 0;
			}
		}

		if (is_last_msg_jump) {
			messages[mkey].end = pos;
			messages[mkey].timeEnd = time;
			messages[mkey].LoopId.last = p->hdr.loop_id;
			messages[mkey].LoopId.lastPos = pos;

			/* Message correctly delivered */
			bw.new_thing(1000 * 8 * p ->msg.len);
			bw.new_time(time);
			prev.msg_id = p->msg.msg_hash;
			prev.msg_id_pos = pos;
		}
		prev.Pos = pos;
		prev.Time = time;
		prev.Frame = *p;
		prev.MKey = mkey;

	}

	void process() {
		std::map<unsigned int, Interval>::iterator it;
		for (it = messages.begin(); it != messages.end(); ++it) {
			Interval be = it->second;

			nrd.new_touch();
			rescheduled.new_touch();
			orphan.new_touch();
			preempted.new_touch();
			o3.new_touch();

			if (be.rescheduled) {
				rescheduled.new_value(be.begin, be.end);
			}

			if (be.begin != 0) {
				if (be.end != 0) {
					if ((be.timeEnd - be.timeBegin)>0){
						mdd.new_value(be.timeEnd - be.timeBegin, be.begin);
					}
					if (be.LoopId.first != be.LoopId.last){
						preempted.new_value(it->first,be.LoopId.firstPos,it->first,be.LoopId.lastPos);
					}
					if (be.timeLoopBegin != 0) {
						if ((be.timeEnd - be.timeLoopBegin)>0){
							papMdd.new_value(be.timeEnd - be.timeLoopBegin, be.end);
							if (prio.find(be.prio)==prio.end()){
								prio[be.prio] = new MaxMinCStats<long long>("Priority "+toString(be.prio),"us");
							}
							prio[be.prio]->new_value(be.timeEnd - be.timeLoopBegin, be.begin);

							/* saves also e-to-e + age */
							int prioWithAge = be.prio + 1000;
							if (prio.find(prioWithAge)==prio.end()){
								prio[prioWithAge] = new MaxMinCStats<long long>("Priority " + toString(be.prio) + " (total)", "ms");
							}
							prio[prioWithAge]->new_value(be.age + (be.timeEnd/1000 - be.timeLoopBegin/1000) , be.begin);

							/* saves also ages alone */
							prioWithAge = be.prio + 2000;
							if (prio.find(prioWithAge)==prio.end()){
								prio[prioWithAge] = new MaxMinCStats<long long>("Priority " + toString(be.prio) + " (age)", "ms");
							}
							prio[prioWithAge]->new_value(be.age, be.begin);
						}
					}
				} else {
					orphan.new_value(be.begin, be.begin);
					nrd.new_value(be.begin, be.begin);
				}
			}
		}
	}
	MaxMinCStats<long long> & getMdd(){
		return mdd;
	}
	void getBegin(){
		prioIt=prio.begin();
	}
	bool hasMore(){
		bool ret=(prioIt!=prio.end());
		return ret;
	}
	int getId(){
		 return prioIt->first;
	}
	MaxMinCStats<long long> & getNextPrioLoop(){
		MaxMinCStats<long long> * ret = prioIt->second;
		prioIt++;
		return *ret;
	}
	MaxMinCStats<long long> & getCTT(){
		return ctt;
	}

	MaxMinCStats<long long> & getCTA(){
		return cta;
	}

	MaxMinCStats<long long> & getCTM(){
		return ctm;
	}

	MaxMinCStats<long long> & getPapMdd(){
		return papMdd;
	}

	ListCStats<long long> & getNrd(){
		return nrd;
	}
	ListCStats<int> & getRescheduled(){
			return rescheduled;
	}
	DoubleListCStats<int> & getPreempted(){
			return preempted;
	}

	DoubleListCStats<int> & getO3(){
			return o3;
	}

	ListCStats<int> & getOrphan(){
		return orphan;
	}

	PlainCStats<int> & getNMsg(){
		return nmsg;
	}
	ThingOverTimeCStats<long long, long long> & getBw(){
		return bw;
	};
	ThingOverTimeCStats<long long, long long> & getAvail(){
		return avail;
	};

	void new_time(long long time) {
		bw.new_time(time);
	}
	void reset() {
		messages.clear();
	}
};

#endif /* STATS_H_ */
