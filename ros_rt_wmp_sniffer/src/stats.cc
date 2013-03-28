/*------------------------------------------------------------------------
 *---------------------           WMPSNIFFER          --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: stas.cc
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


#include "stats.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

void initStats(Stats * s, const char * name, const char* unit) {
	s->min = (int) pow(2.0, 32.0) - 1;
	s->max = 0;
	s->mean = 0;
	s->pos_max = 0;
	s->pos_min = 0;
	s->sum = 0;
	s->bookmark = 0;
	s->idx = 0;
	sprintf(s->name,"%s",name);
	sprintf(s->unit,"%s",unit);
	s->v.clear();
	s->v2.clear();
}

void initStats(Stats * s, const char * name, const char* unit, const char* child_name) {
	sprintf(s->child_name, "%s", child_name);
	initStats(s, name, unit);
}

/* Min/Max values */
void updateStat(Stats * s, int val, int pos) {
	s->idx++;
	s->sum += val;
	if (val > s->max) {
		s->max = val;
		s->pos_max = pos;
	}
	if (val < s->min) {
		s->min = val;
		s->pos_min = pos;
	}
	s->v.push_back(val);
}

/* Value/Pose */
void addElement(Stats * s, int value, int pos) {
	s->sum += value;
	s->v.push_back(value);
	s->v2.push_back(pos);
	s->idx++;
}

/* Value and num of elements */
void updateValueStats(Stats * s, int value, int index) {
	while ((index + 1) > s->v.size()) {
		s->v.push_back(0);
		s->v2.push_back(0);
	}
	s->v.at(index) += value;
	s->v2.at(index)++;
	s->idx++;
}

/* Value and Value */
void updateValueStats2(Stats * s, int value, int index) {
	while ((index + 1) > s->v.size()) {
		s->v.push_back(0);
		s->v2.push_back(0);
	}
	s->v.at(index)  += value;
	s->v2.at(index) += value;
	s->idx+=value;
}
