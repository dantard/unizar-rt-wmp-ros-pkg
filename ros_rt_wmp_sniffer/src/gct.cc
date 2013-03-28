/*------------------------------------------------------------------------
 *---------------------           WMPSNIFFER          --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: gct.cc
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
#include "gct.h"
#include <cmath>
#include <cstdlib>
#include "misc.h"
static int previous_serial = 0, gct_initd = 0, act_gct = 0,max_consecutive = 0,idx1;

int greatest_clean_time_check(wmpFrame * p) {
    if (gct_initd == 0) {
        idx1 = p->hdr.serial;
        previous_serial = idx1 - 1;
        gct_initd = 1;
    }
    if (p->hdr.serial != previous_serial + 1) {
        if (abs(p->hdr.serial - idx1) > max_consecutive) {
            max_consecutive = abs(p->hdr.serial - idx1);
        }
        idx1 = p->hdr.serial;
    }
    act_gct = MAX(abs(p->hdr.serial - idx1), max_consecutive);
    previous_serial = p->hdr.serial;
    return max_consecutive;
}

int get_actual_gct(int *best, int *act) {
    *best = act_gct;
    *act = abs(previous_serial - idx1);
    return act_gct;
}

void reset_actual_gct(){
	act_gct=0;
	previous_serial=idx1=0;
}
