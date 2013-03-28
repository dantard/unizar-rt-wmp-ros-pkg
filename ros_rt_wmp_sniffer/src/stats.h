/*------------------------------------------------------------------------
 *---------------------           WMPSNIFFER          --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: stats.h
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


#ifndef STATS_H_
#define STATS_H_
#include <vector>

struct Stats {
	char name[64];
	char child_name[64];
	char unit[16];
	int idx;
	long long sum;
	float mean;
	long long min;
	long long max;
	int pos_min;
	int pos_max;
	long long bookmark;
	std::vector<int> v;
	std::vector<int> v2;
};

void initStats(Stats * s, const char * name, const char* unit);
void initStats(Stats * s, const char * name, const char* unit, const char* child_name);
void updateStat(Stats * s, int val, int pos);
void addElement(Stats * s, int value, int pos);
void updateValueStats(Stats * s, int value, int index);
void updateValueStats2(Stats * s, int value, int index);

#endif /* STATS_H_ */
