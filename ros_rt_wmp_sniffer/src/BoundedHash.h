/*------------------------------------------------------------------------
 *---------------------           WMPSNIFFER          --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: BoundedHash.h
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
#ifndef BOUNDEDHASH_H_
#define BOUNDEDHASH_H_

#include <ext/hash_map>
#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <pthread.h>
#include <semaphore.h>
#include <cassert>
using namespace std;
using __gnu_cxx::hash_map;



template <class S, class T, class R> class BoundedHash {
	typedef typename multimap<S,S>::iterator mmit;
private:

	struct bh_ts_key{
		S first, second, ds;
		R sort_key;
	};

	static bool sort_fun(bh_ts_key i, bh_ts_key j) {
		if (i.sort_key < 0 || j.sort_key < 0) assert(0);

		return (i.sort_key <= j.sort_key);
	}

	pthread_mutex_t mtxs;
	sem_t sem;
	hash_map<S,T> hm;
	multimap<S,S> mm;
	int size,buffer;
	std::vector<bh_ts_key> v;
	bool strict;

	bool erase(int vector_idx){
		bh_ts_key elem = v.at(vector_idx);
		S key = elem.first;
		S key2 = elem.second;
		//cout << "kill elem first key : "<< elem.first << "\n";
		//cout << "kill elem sec   key2: "<< elem.second << "\n";
		v.erase(v.begin()+vector_idx);
		hm.erase(key);
		std::pair< mmit, mmit > it;
		bool deleted=true;
		while (deleted){
			deleted=false;
			it = mm.equal_range(key2);
			for (mmit it2 = it.first; it2 != it.second;++it2) {
				if (it2->second == key) {
					mm.erase(it2);
					deleted=true;
					break;
				}
			}
		}
		return true;
	}
public:

	BoundedHash(int size, int buffer){
		this->size=size;
		this->strict = false;
		pthread_mutex_init(&mtxs,0);
		sem_init(&sem,0,0);
	}

	BoundedHash(){
		this->size=100;
		this->buffer=50;
		this->strict = false;
		pthread_mutex_init(&mtxs,0);
		sem_init(&sem,0,0);
	}
	void setStrict(bool _strict){
		strict = _strict;
	}
	int getSize(){
		//cout << "hm:" << hm.size() << " mm: " << mm.size();
		return v.size();
	}

	bool setBufferSize(int size){
		this->buffer=size;
		return true;
	}

	bool setSize(int size){
		if (v.size()<size){
			this->size=size;
			return true;
		}else{
			return false;
		}
	}

	virtual ~BoundedHash(){}

	void lock(){
		pthread_mutex_lock(&mtxs);
	}
	void unlock(){
		pthread_mutex_unlock(&mtxs);
	}

	void drain(){
		int nelem=0;
		if (v.size() > buffer){
			nelem = buffer;
		} else{
			nelem = v.size();
		}
		for (int i = 0 ; i < nelem; i++){
			sem_post(&sem);
		}
	}

	T& insert(S key, S key2, R sort_key){
		bool present=false;
		for (int i=0; i<v.size(); i++){
			present |= (v.at(i).first==key);
		}
		bh_ts_key p;
		p.first=key;
		p.second=key2;
		p.sort_key=sort_key;
		if (not present){
			mm.insert(pair<S,S>(key2, key));
			v.push_back(p);
			//cout << "Inserting elem : "<< v.size() << "\n";
			if (v.size() == size && !strict){
				erase(0);
			}else{
				if (v.size()>buffer) sem_post(&sem);
			}
		}
		/* DEBUG if (v.size()>10){
			for (int i=0;i<10;i++){
				fprintf(stderr,"Pos#%d Key1:%d key2:%d\n",i,v.at(i).first,v.at(i).second);
			}
		}*/
		return hm[key];
	}

	void sort(){
		std::sort(v.begin(),v.end(),sort_fun);
	}

	T& get(S key){
		return hm[key];
	}

	S getKey(S key2, int idx=0){
		pthread_mutex_lock(&mtxs);
		std::pair< mmit, mmit > it= mm.equal_range(key2);
		mmit it2 = it.first;
		pthread_mutex_unlock(&mtxs);
		advance(it2,idx);
		return it2->second;
	}

	int countk2(S key2){
		pthread_mutex_lock(&mtxs);
		int c =mm.count(key2);
		pthread_mutex_unlock(&mtxs);
		return c;
	}

	T pop(){
		sem_wait(&sem);
		pthread_mutex_lock(&mtxs);
		//std::sort(v.begin(),v.end(),sort_fun);
		bh_ts_key elem = v.at(0);
		int key = elem.first;
		T t = hm[key];
		erase(0);
		pthread_mutex_unlock(&mtxs);
		return t;
	}

	T pop(int key){
		pthread_mutex_lock(&mtxs);
		int idx=-1;
		for (int i=0; i<v.size(); i++){
			if (key==v.at(i).first){
				idx=i;
			}
		}
		T t = hm[key];
		if (idx >= 0) erase(idx);

		pthread_mutex_unlock(&mtxs);
		return t;
	}

	bool find(S key){
		bool present=false;
		for (int i=0; i<v.size(); i++){
			present |= (v.at(i).first==key);
		}
		return present;
	}

	void sleep(){
		if (v.size()>this->size){
			usleep(v.size()-this->size);
		}
	}

	void clear(){
		int nSignals = (v.size()-buffer)>0?(v.size()-buffer):0;
		for (int i = 0 ; i <nSignals  ; i++){
			pop();
		}
		v.clear();
		hm.clear();
		mm.clear();
	}
	bool blocked(){
		return (not(v.size()>buffer));
	}
	void signal(){
		sem_post(&sem);
	}
};

#endif /* BOUNDEDHASH_H_ */
