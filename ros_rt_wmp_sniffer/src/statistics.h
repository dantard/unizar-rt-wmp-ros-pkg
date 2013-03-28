/*------------------------------------------------------------------------
 *---------------------           WMPSNIFFER          --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: statistics.h
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

#ifndef STATISTICS_H_
#define STATISTICS_H_

#include <stdio.h>
#include <string.h>
#include <vector>
#include <math.h>
#include "wmp_config.h"

#include "core/include/frames.h"
#include "window2.hh"
//using namespace std;

#define NONE -1
#define MIN_V -2
#define MAX_V -3

void statistics_init() ;
void statistics_new_frame(wmpFrame * r, long long time, int i, int bytes, simData_Hdr * sdh=NULL);
void statistics_publish(window2 * w);
void statistics_get_fez(int & min, int & max);
int statistics_from_file(char * filename, int & begin, int & end);
void stat_plot_tg(int i);
void stat_plot_hist(int i, int nbins, double zoom);
bool stat_ask_plotter(int i);
void stat_write_to_file(int i);
#endif /* STATISTICS_H_ */
