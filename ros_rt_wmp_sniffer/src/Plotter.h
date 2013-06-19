/*------------------------------------------------------------------------
 *---------------------           WMPSNIFFER          --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: Plotter.h
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


#ifndef PLOTTER_H_
#define PLOTTER_H_
#include <stdio.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <algorithm>
#include "Stats.h"
#include <ext/hash_map>
#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <pthread.h>
#include <semaphore.h>
#include <assert.h>

using namespace std;
template <class Q, class T>
class Plotter {
std::vector<Q * > v;
std::vector<T> v1,v2;
std::vector<std::string> names;
public:
	void write_to_file(int id) {
		std::ofstream fdata;
		std::string sname = names.at(id);
		sname += ".data";
		for (int i = 0; i < sname.length(); i++) {
			if (sname[i] == ' ' || sname[i] == ':' || sname[i] == ' '
					|| sname[i] == '#') {
				sname[i] = '_';
			}
		}
		std::cout << "Writing " << sname.c_str() << std::endl;
		fdata.open(sname.c_str());
		Q * elem = v.at(id);
		//fdata << name.at(i) << std::endl << std::endl;
		for (int j = 0; j < elem->getCount(); j++) {
			fdata << elem->get(j) << "\n"; // <<  << "::" << elem->getName() << std::endl;
		}
		fdata.close();
	}

	void writeAll(std::string name) {

		//TODO:
		return;
		int res = system("mkdir data");
		if (res!=0){
			fprintf(stderr,"Error creating 'data' directory\n");
		}
		int pos = name.find_last_of('/');
		if (pos > 0){
			name = name.substr(pos);
			pos = name.find_last_of('.');
			name = name.substr(0,pos);
		}
		name += "_";
		for (int i = 0; i< v.size(); i++){
			std::ofstream fdata;
			std::string sname = "data/" + name;
			sname += names.at(i);
			sname += ".data";
			for (int j=0; j< sname.length();j++){
				if (sname[j] == ' ' || sname[j] == ':' || sname[j] == ' ' || sname[j] == '#' ){
					sname[j] = '_';
				}
			}

			fdata.open(sname.c_str());

			Q * elem = v.at(i);
			//fdata << name.at(i) << std::endl << std::endl;
			for (int j = 0; j < elem->getCount(); j++) {
				fdata << elem->get(j) << "\n";// <<  << "::" << elem->getName() << std::endl;
			}
			fdata.close();
		}

	}

	Plotter(){

	}

	int subscribe(Q * elem, std::string s= "***"){
		v.push_back(elem);
		names.push_back(s);
		return (v.size()-1);
	}

	void plot(int idx){
		if (idx < v.size()){
			Q * elem = v.at(idx);
			std::ofstream fdata;
			fdata.open (".dataplot");
			for (int i = 0 ; i< elem->getCount();i++){
				fdata << elem->getPos(i) << " " << elem->get(i) << std::endl;
			}
			fdata.close();
			FILE * f = popen("gnuplot","w");
			string s = "set xlabel 'Sample'\n";
			fprintf(f,"%s",s.c_str());
			s = "set ylabel '"+ elem->getName()+" (ms)'\n";
			fprintf(f,"%s",s.c_str());
			//s = "plot '.dataplot' w lines title '" + elem->getName() + "'\n";
			s = "plot '.dataplot' title '" + elem->getName() + "'\n";
			fprintf(f,"%s",s.c_str());
			fprintf(stderr,"*** %s",s.c_str());
			fflush(f);
		}
	}

	void plotHist(int idx, int nbins, double zoom){
		if (idx < v.size()) {
			Q * elem = v.at(idx);
			T mean=0;
			for (int i = 0; i < elem->getCount(); i++) {
				v1.push_back(elem->get(i));
				mean+=elem->get(i);
			}
			mean = mean / v1.size();
			std::sort(v1.begin(), v1.end());

			v1.resize(v1.size()*zoom);

			double min = v1.at(0);
			double max = v1.at(v1.size() - 1);
			if (nbins == 0){
				nbins = (max/mean) /1.5;
			}
			v2.resize(nbins + 1);

			double bin = (max - min) / (double) nbins;
			for (int i = 0; i < v1.size(); i++) {
				int sbin = (int) (( (double) v1.at(i) - min) / bin);
				v2.at(sbin)++;
			}
			std::ofstream fdata;
			fdata.open (".dataplot");
			for (int i = 0; i < v2.size(); i++) {
				fdata << double((min+i*bin)/1000.0) << " " << v2.at(i) << std::endl;
			}
			fdata.close();
			FILE * f = popen("gnuplot","w");
			string s = "set xlabel 'Time (ms)'\n";
			fprintf(f,"%s",s.c_str());
			s = "set ylabel 'Occurrences'\n";
			fprintf(f,"%s",s.c_str());
			s = "plot '.dataplot' w boxes title '" + elem->getName() + "'\n";// + std::endl;
			fprintf(f,"%s",s.c_str());
			fflush(f);
		}
	}
	void clear(){
		v.clear();
	}
	bool hasThis(int i){
		bool ret = (i < v.size() && i >= 0);
		return ret;
	}
	virtual ~Plotter(){

	}
};

#endif /* PLOTTER_H_ */
