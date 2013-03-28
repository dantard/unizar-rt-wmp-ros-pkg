/*------------------------------------------------------------------------
 *---------------------           RT-WMP              --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: ./src/core/include/manage.h                                     
 *  Authors: Danilo Tardioli                                              
 *  ----------------------------------------------------------------------
 *  Copyright (C) 2000-2010, Universidad de Zaragoza, SPAIN
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



#ifndef MANAGE__H
#define MANAGE__H
#include "config/compiler.h"
#include "queues.h"
#include "nstat.h"
#include "rssi_average.h"
#include "wmp_com.h"
#include "wmp_misc.h"
#include "lqm.h"
#include "dijkstra.h"
#include "prim.h"
#include "core/include/frames.h"
#include "core/interface/wmp_interface.h"


int evaluate_token(wmpFrame * buffer);
int evaluate_authorization(wmpFrame * t);
int evaluate_message(wmpFrame * t);
int evaluate_foreign(wmpFrame * t);

int manage_token_expired_timeout(wmpFrame  * p);
int manage_authorization_expired_timeout(wmpFrame * t);
int manage_message_expired_timeout(wmpFrame * t);


int create_new_token(wmpFrame* t);
int create_authorization(wmpFrame * t);
int create_message(wmpFrame * p);
int enqueue_message(wmpFrame * p);




#endif

