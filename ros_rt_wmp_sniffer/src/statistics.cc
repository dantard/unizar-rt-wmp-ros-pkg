/*------------------------------------------------------------------------
 *---------------------           WMPSNIFFER          --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: statistics.cc
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

#include <stdio.h>
#include <string.h>
#include <vector>
#include <math.h>
#include <assert.h>
#include <iostream>
#include "wmp_config.h"

#include "core/include/frames.h"
#include "bridge.hh"
#include "core/interface/Msg.h"
#include "window2.hh"
#include "statistics.h"
#include "basic_io.h"
#include "stats.h"
#include "graphs.h"
#include <map>
#include "Stats.h"
#include "Plotter.h"
using namespace std;

ThingOverTimeCStats<long long, int> timespan("Interval","us");

PlainCStats<int> tkns("Tokens","");
PlainCStats<int> msgs("Messages","");
PlainCStats<int> auths("Authorizations","");

PlainCStats<long long> th_wc("Calculated WC Loop","us");
PlainCStats<long long> th_bc_wc("Calculated WC BC Loop","us");

MaxMinCStats<long long> paps("PAP Duration","us");
MaxMinCStats<long long> loops("LOOP Duration","us");
MaxMinCStats<long long> wc_loops("WC LOOP Duration","us");
MaxMinCStats<long long> inter_token("Inter-Token","us");
MaxMinCStats<long long> inter_message("Inter-Message","us");
MaxMinCStats<long long> inter_auth("Inter-Auth","us");
MaxMinCStats<long long> inter_fore("Inter-Foreign","us");
MaxMinCStats<long long> loop_length("Loop length","hops");
MaxMinCStats<long long> bc_loop("BC Loop duration","us");

ListCStats<long long> reinserted("Reinserted","Serial");
ListCStats<long long> retries("Retries","Serial");
ListCStats<int> ncs("Not Consecutive Serial","Id");
ListCStats<int> incongruent("Incongruent Cases","Id");
ListCStats<int> drops("Drops","Drop Serial");
ListCStats<int> ett("ETT","Extra time (ms) ");
ListCStats<int> plrx("Possibly Lost in RX","Serial");
ListCStats<int> foreign("Foreign Frames","Id");
ListCStats<long long> stops("Stops","ms");
ListCStats<long long> qos_stops("Messages Stop","Serial");

IntervalCStats<int> efz("Max Error Free Zone","Frames");
IntervalCStats<int> mmz("Max Message Zone","Frames");

Plotter< MaxMinCStats<long long>, long long > plotter;

std::map<int, StatsFlow *> flows;

std::map<unsigned int, long long> bc_map;
long long last_ptime = 0, first_time;
int last_pos = 0;
bool first = false;
char last_frame[2500];
int total_frames = 0;

#define PLAIN_RTWMP 	0
#define QOS_RTWMP 		97

bool first_frame = true;



/* STATISTICS >>> */
static int nnodes = 0, from, to;
static window2 * w2;

int statistics_from_file(char * filename, int & begin, int & end){
	nnodes=io_open_sim_data(filename);
	int n=0;
	if (nnodes>0) {
		n=io_get_sim_data_num_of_elements();
	} else {
		return -1;
	}
	if (begin<0 || to <0){
		from=0;
		to=n;
	}else{
		from=begin;
		to=end;
	}
	io_go_to(from);
	char fdata[2700];

	simData_Hdr * p = (simData_Hdr *) fdata;
	wmpFrame * r = (wmpFrame *) (fdata + sizeof(simData_Hdr));
	long long base_time = 0;

	int idx=0;
	fprintf(stderr,"Generating Statistics...\n");
	first_frame = true;
	for (int i=from; i<to; i++) {
		int bytes=io_read_next_sim_data(&fdata[0]);
		if (i==from) base_time=p->time;
		if (bytes > 0){
			statistics_new_frame(r,p->time-base_time,i,bytes,p);
			printf("Analizing Data %3.1f %c \r",100.0*double(i-from)/double(to-from),'%');
		}else{
			break;
		}
		idx++;
	}

	efz.done();
	mmz.done();

	begin=from;
	end=from+idx;
	return idx;
}

void statistics_get_fez(int & min, int & max){
	CStats<int>::stat_t mi = efz.getIntervalMin();
	CStats<int>::stat_t ma = efz.getIntervalMax();
	min = mi.pos;
	max = ma.pos;
}



int hf(int key)
{
  int c2=0x27d4eb2d; // a prime or an odd constant
  key = (key ^ 61) ^ (key >> 16);
  key = key + (key << 3);
  key = key ^ (key >> 4);
  key = key * c2;
  key = key ^ (key >> 15);
  return key;
}


static int lastForeignTime = 0;
long long lastIat;
unsigned long long old_qosPh_serial = 0;
FILE * outf;
void statistics_init() {
	outf= fopen("outf.txt","w+");
	std::map<int, StatsFlow * >::iterator iter;
	for (iter = flows.begin(); iter != flows.end(); ++iter) {
		delete iter->second;
	}
	flows.clear();

	flows[0] = new StatsFlow(-1,-1,0);
	flows[1] = new StatsFlow(-1,-1,1);

	tkns.init();
	msgs.init();
	auths.init();
	th_wc.init();
	th_bc_wc.init();

	paps.init();
	loops.init();
	wc_loops.init();

	inter_token.init();
	inter_message.init();
	inter_auth.init();
	inter_fore.init();

	loop_length.init();

	reinserted.init();
	retries.init();
	ncs.init();
	incongruent.init();
	drops.init();
	ett.init();
	plrx.init();

	efz.init();
	mmz.init();
	timespan.init();
	foreign.init();
	stops.init();
	qos_stops.init();

	bc_loop.init();

	lastForeignTime = 0;
}

bool last_pap_jump(wmpFrame * p){
	int sum = 0, idx = 0;
	char * pnt = wmp_get_frame_routing_pointer(p, nnodes);
	for (int j = 0; j < nnodes; j++) {
#ifdef WMP_ROUTING_tree
		unsigned char reached = *(pnt+nnodes+j);
#else
		unsigned char reached = pnt[j * (nnodes + 1)];
#endif
		if (!(reached & 128)) {
			sum += 1;
			idx = j;
		}
	}

	if (sum == 1 && idx == p->hdr.to) {
		return true;
	}else{
		return false;
	}

}

bool new_token(wmpFrame * p){
	return false;//p->tkn.new_token;
}

void statistics_new_frame(wmpFrame * p, long long ptime, int pos, int bytes, simData_Hdr * sdh) {
	if (pos == from){
		last_ptime = ptime;
		last_pos = pos;
		first_time = ptime;
	}

	if (!sdh->is_wmp){
		foreign.new_value(pos,pos);
		if (lastForeignTime != 0){
			inter_fore.new_value(last_ptime - lastForeignTime, pos);
		}
		lastForeignTime = ptime;
		return;
	}
	if (ptime - last_ptime > 400000){
		stops.new_value(ptime/1000 - last_ptime/1000,pos);
	}
	qos_stops.new_touch();
	stops.new_touch();
	foreign.new_touch();
	ncs.new_touch();
	incongruent.new_touch();
	retries.new_touch();
	reinserted.new_touch();
	drops.new_touch();
	plrx.new_touch();
	ett.new_touch();
	bc_loop.new_touch();

	wmpFrame * q = (wmpFrame *) last_frame;
	efz.new_value(pos,pos);
	timespan.new_time(ptime);

	/* GENERAL */
	if (p->hdr.serial != q->hdr.serial + 1 && !first_frame){
		ncs.new_value(pos,pos);
		loop_length.aux2 = 0;
		efz.reset();
	}

	first_frame = false;

	if (p->hdr.serial == q->hdr.serial + 2){
		plrx.new_value(q->hdr.serial+1,pos);
	}

	if (p->hdr.type == DROP_TOKEN){
		drops.new_value(p->drop.drop_serial,pos);
		efz.reset();
	}

	if (q->hdr.type == AUTHORIZATION && p->hdr.type == TOKEN){
		incongruent.new_value(last_pos,last_pos);
		//efz.reset();
	}

	if (q->hdr.type == TOKEN && p->hdr.type == TOKEN){
		inter_token.new_value(ptime-last_ptime,pos);
	}
	if (q->hdr.type == AUTHORIZATION && p->hdr.type == AUTHORIZATION){
		inter_auth.new_value(ptime-last_ptime,pos);
	}
	if (q->hdr.type == MESSAGE && p->hdr.type == MESSAGE){
		inter_message.new_value(ptime-last_ptime,pos);
	}

	if (p->hdr.retries !=0 ){
		retries.new_value(p->hdr.serial,pos);
		loop_length.aux2 = 0;
	}

	if (sdh->reinserted == 32){
		reinserted.new_value(p->hdr.serial,pos);
	}

	if (p->hdr.type == TOKEN){
		tkns.new_value(1);
		bool is_new_token = new_token(p);
		bool is_last_pap_jump = last_pap_jump(p);

		if (is_new_token){
			paps.bookmark = last_ptime;
			paps.aux1 = 1; /* to see if pap is interrupted */
			loops.bookmark = last_ptime;
			loop_length.aux1 = 0;
			loop_length.aux2 = 1;
			if (mmz.aux1 == 0){
				mmz.reset();
			}
			mmz.aux1 = 0;
		}
		if (paps.aux1==1 && is_last_pap_jump){
			paps.new_value(ptime-paps.bookmark,pos);
		}
	}else{
		paps.aux1 = 0;
	}

	loop_length.aux1++;

	if (p->hdr.type == AUTHORIZATION){
			auths.new_value(1);

	}

	if (p->hdr.type == MESSAGE){
		msgs.new_value(1);

		bool is_last_msg_jump = (p->msg.dest & (1 << p->hdr.to));

		/* Flows */
		//if (is_last_msg_jump) {
			int flow_id = p->hdr.to * 100 + p->msg.src + 1;
			int pernode_id = p->msg.src + 10000;

			if (flows.find(flow_id) == flows.end()) {
				flows[flow_id] = new StatsFlow(p->msg.src, p->hdr.to, 0, first_time);
			}

			if (flows.find(pernode_id) == flows.end()) {
				flows[pernode_id] = new StatsFlow(p->msg.src, -1, 0, first_time);
			}
		//}
		/* Flows */

		//if (is_last_msg_jump) {
			if (p->hdr.retries == 0) {

				if (loop_length.aux2 == 1) {
					loop_length.new_value(loop_length.aux1, pos);
					loops.new_value(ptime - loops.bookmark, pos);
					if (loop_length.aux1 == ((2 * nnodes - 3) + (nnodes - 1)
							+ (nnodes - 1))) {
						wc_loops.new_value(ptime - loops.bookmark, pos);
					}
				}
			}
			/* Message correctly delivered */
			mmz.new_value(pos, pos);
			mmz.aux1 = 1;
		//}

	}

	std::map<int, StatsFlow * >::iterator iter;
	for (iter = flows.begin(); iter != flows.end(); ++iter) {
		iter->second->new_frame(p,ptime,pos);
	}

	/* next_loop_save */
	last_ptime = ptime;
	last_pos = pos;
	total_frames++;
	memcpy(last_frame,p,bytes);
}

template <class T>
void publish(MaxMinCStats<T> mms, int precision=0, int uid=-1){
	if (mms.isValid()){
		if (uid==-1){

			w2->row_create(mms.getName()+ " (" + toString(mms.getCount())+ ")",toString(mms.getMean(),precision)+" " + mms.getUnit() , -1, uid);
		} else {
			w2->row_create(mms.getName()+"*"+ " (" + toString(mms.getCount()) + ")",toString(mms.getMean(),precision)+" " + mms.getUnit(), -1, uid);
		}
		typename CStats<T>::stat_t val = mms.getMax();
		w2->subSectionBegin();
		w2->row_create("Max",toString(val.value,precision)+" " + mms.getUnit(),val.pos);
		val = mms.getMin();
		w2->row_create("Min",toString(val.value,precision)+" " + mms.getUnit(),val.pos);
		w2->subSectionEnd();
	}
}

template <class T>
void publish(IntervalCStats<T> mms, int precision=0){
	if (mms.isValid()){
		w2->row_create(mms.getName(),toString(mms.getIntervalCount(),precision)+" " + mms.getUnit());
		typename CStats<T>::stat_t val = mms.getIntervalMin();
		w2->subSectionBegin();
		w2->row_create("From",toString(val.value,precision)+" " + mms.getUnit(),val.pos);
		val = mms.getIntervalMax();
		w2->row_create("To",toString(val.value,precision)+" " + mms.getUnit(),val.pos);
		w2->subSectionEnd();
	}
}

template <class T>
void publish(DoubleListCStats<T> lcs, int precision=0){
	w2->row_create(lcs.getName(),toString(lcs.getCount()) + "  (" + toString(lcs.getPercentage(),3) + "%)");
	w2->subSectionBegin();
	typename CStats<T>::stat_t val1,val2;
	for (int i=0; i< lcs.getCount();i++){
		lcs.getElem(i,val1,val2);
		w2->row_create(lcs.getChildName(),toString(val1.value,precision),-1);
		w2->subSectionBegin();
		w2->row_create("Begin",toString(val1.pos,precision),val1.pos);
		w2->row_create("End",toString(val2.pos,precision),val2.pos);
		w2->subSectionEnd();
	}
	w2->subSectionEnd();
}


template <class T>
void publish(ListCStats<T> lcs, int precision=0){
	w2->row_create(lcs.getName(),toString(lcs.getCount()) + "  (" + toString(lcs.getPercentage(),3) + "%)");
	w2->subSectionBegin();
	typename CStats<T>::stat_t val;
	for (int i=0; i< lcs.getCount();i++){
		val = lcs.getElem(i);
		w2->row_create(lcs.getChildName(),toString(val.value,precision),val.pos);
	}
	w2->subSectionEnd();
}

template <class T>
void publish(PlainCStats<T> pcs, int precision=0){
	w2->row_create(pcs.getName(),toString(pcs.getSum(),precision) + " " + pcs.getUnit());
}
template <class T,class P>

void publish(ThingOverTimeCStats<T,P> tot, int precision=0){
	w2->row_create(tot.getName(),toString(tot.getToT(),precision) + " " + tot.getUnit());
}
void stat_plot_tg(int i){
	plotter.plot(i);
}
void stat_write_to_file(int i){
	plotter.write_to_file(i);
}
void stat_plot_hist(int i, int nbins, double zoom){
	plotter.plotHist(i, nbins, zoom);
}

void statistics_publish(window2 * w) {
	w2=w;

	plotter.clear();
	w2->root();

	int m = timespan.getInterval()/(1000*1000*60);
	int s = (timespan.getInterval()%(1000*1000*60))/(1000*1000);
	int ms = (timespan.getInterval()%(1000*1000*60))%(1000*1000)/1000;
	th_wc.new_value((2*nnodes-3)*inter_token.getMean()+(nnodes-1)*inter_auth.getMean()+(nnodes-1)*inter_message.getMean());
	th_bc_wc.new_value((4*nnodes-9)*inter_token.getMean());

	w2->row_create("Trace Duration",toString(m)+"m "+toString(s)+"s " + toString(ms)+"ms");
	w2->root();
	w2->row_create("General","");
	w2->subSectionBegin();
		publish(tkns);
		publish(auths);
		publish(msgs);
		publish(foreign);
		w2->row_create("InterFrame","");
		w2->subSectionBegin();
			publish(inter_token, 0, plotter.subscribe(&inter_token,w2->getCurrentRowName()));
			publish(inter_auth, 0, plotter.subscribe(&inter_auth,w2->getCurrentRowName()));
			publish(inter_message, 0, plotter.subscribe(&inter_message,w2->getCurrentRowName()));
			publish(inter_fore, 0, plotter.subscribe(&inter_fore,w2->getCurrentRowName()));

	w2->root();
	w2->row_create("Duration","");
	w2->subSectionBegin();
		publish(paps,0,plotter.subscribe(&paps,w2->getCurrentRowName()));
		publish(loops,0,plotter.subscribe(&loops,w2->getCurrentRowName()));
		publish(wc_loops,0,plotter.subscribe(&wc_loops,w2->getCurrentRowName()));
//		publish(th_wc, 0);
//		publish(bc_loop,0,plotter.subscribe(&bc_loop,w2->getCurrentRowName()));
//		publish(th_bc_wc, 0);
	w2->root();
	w2->row_create("Errors","");
	w2->subSectionBegin();
		publish(ncs);
		//publish(incongruent);
		publish(retries);
		publish(drops);
		publish(plrx);
		//publish(stops);
	w2->root();
//	w2->row_create("Various","");
//	w2->subSectionBegin();
//		publish(loop_length,5,plotter.subscribe(&loop_length,w2->getCurrentRowName()));
//		publish(ett);
//		publish(reinserted);
	w2->root();
	w2->row_create("Intervals","");
	w2->subSectionBegin();
		publish(efz);
		//publish(mmz);

		std::map<int, StatsFlow * >::iterator iter;

//	w2->root();
//	w2->row_create("Cpu Time","");
//	w2->subSectionBegin();
//	for (iter = flows.begin(); iter != flows.end(); ++iter) {
//		if (!iter->second->is_qos()) {
//			if (iter->first == 0) {
//				w2->row_create("Global Flow", "Real Time");
//				w2->subSectionBegin();
//				publish(iter->second->getCTT(), 2);
//				publish(iter->second->getCTA(), 2);
//				publish(iter->second->getCTM(), 2);
//				w2->subSectionEnd();
//			} else if (iter->first >= 10000) {
//				w2->row_create("Node # " + toString(iter->second->get_src()),
//						"", iter->second->begin());
//				w2->subSectionBegin();
//				publish(iter->second->getCTT(), 2);
//				publish(iter->second->getCTA(), 2);
//				publish(iter->second->getCTM(), 2);
//				w2->subSectionEnd();
//			}
//		}
//	}

	w2->root();
	w2->row_create("Flows","");
	w2->subSectionBegin();
//	w2->row_create("HRT","");
//	w2->subSectionBegin();
	int i, size = flows.size();
	for (iter = flows.begin(), i = 1; iter != flows.end(); ++iter,i++) {
		printf("Processing Data %3.1f %c\r",100.0*double(i)/double(size),'%');
		fflush(stdout);
		if (!iter->second->is_qos()) {
			if (iter->first == 0) {
				w2->row_create("Global Flow", "");
			} else if (iter->first < 10000){
				w2->row_create("Flow :: Src: " + toString(
						iter->second->get_src()) + " Dst: " + toString(
						iter->second->get_dst()),"", iter->second->begin());
			} else {
				w2->row_create("Node # " + toString(
						iter->second->get_src()),"", iter->second->begin());
			}
			w2->subSectionBegin();
			iter->second->process();
			publish(iter->second->getNMsg(), 2);
			publish(iter->second->getBw(), 2);
			publish(iter->second->getNrd(), 2);
			publish(iter->second->getMdd(), 0, plotter.subscribe(&iter->second->getMdd(),w2->getCurrentRowName(),1000));
			publish(iter->second->getPapMdd(), 0, plotter.subscribe(&iter->second->getPapMdd(),w2->getCurrentRowName(),1000));
//			publish(iter->second->getRescheduled(), 2);
//			publish(iter->second->getOrphan(), 2);
//			publish(iter->second->getPreempted(), 2);



//			if (iter->first != 0) {
//				publish(iter->second->getO3(), 2);
//			}

			w2->row_create("By Priority (Ages)", "");
			w2->subSectionBegin();
			iter->second->getBegin();
			while (iter->second->hasMore()){
				int id = iter->second->getId();
				MaxMinCStats<long long> & npl = iter->second->getNextPrioLoop();
				if (id >= 2000){
					publish(npl, 2,plotter.subscribe(&npl,w2->getCurrentRowName(),1000));
				}
			}
			w2->subSectionEnd();

			w2->row_create("By Priority", "");
			w2->subSectionBegin();
			iter->second->getBegin();
			while (iter->second->hasMore()){
				int id = iter->second->getId();
				MaxMinCStats<long long> & npl = iter->second->getNextPrioLoop();
				if (id < 1000){
					publish(npl, 2,plotter.subscribe(&npl,w2->getCurrentRowName(),1000));
				}
			}
			w2->subSectionEnd();

			w2->row_create("By Priority + age", "");
			w2->subSectionBegin();
			iter->second->getBegin();
			while (iter->second->hasMore()) {
				int id = iter->second->getId();
				MaxMinCStats<long long> & npl = iter->second->getNextPrioLoop();
				if (id >= 1000){
					publish(npl, 2, plotter.subscribe(&npl,w2->getCurrentRowName(),1000));
				}
			}
			w2->subSectionEnd();
			w2->subSectionEnd();
		}
	}
	w2->subSectionEnd();
	printf("\nDone.\n");

	fflush(stdout);
	plotter.writeAll(w->getFileName());
}

bool stat_ask_plotter(int i){
	return plotter.hasThis(i);
}
