/*------------------------------------------------------------------------
 *---------------------           RT-WMP              --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: ./src/core/include/wmp_com.h                                    
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



#ifndef WMP_COM__H
#define WMP_COM__H

#include "wmp_config.h"
#include "config/compiler.h"
#include "wmp_com.h"
#include "lqm.h"
#include "rssi_average.h"
#include "global.h"
#include "nstat.h"
#include "global.h"
#include "wmp_misc.h"
#include "core/include/frames.h"
void encode_routing_info(wmpFrame * t);
void decode_routing_info(wmpFrame * t); 

int wmpWaitAck(wmpFrame* p);
int wmpReceive(wmpFrame* p);
int wmpSend(wmpFrame* t);
void wmpSendAck(wmpFrame * p);

void wmpUnlockRead(int rtnCode);
void wmpTest(void);
void wmpSendDrop(wmpFrame * t);
void wmpUpdateRssi(wmpFrame * t);
int wmpUpdateReceivedRssi(wmpFrame* q);
int wmpUpdateAcknowkedgedRssi(wmpFrame* q);
int wmpInterpretReceived(wmpFrame **p,wmpFrame **q);
int wmpInterpretAck(wmpFrame **p,wmpFrame **q);
int vigilant_sleep(wmpFrame * p, wmpFrame * q);

void wmp_send_setup(wmpFrame * p);
int wmp_send_retry(wmpFrame * p);
void initComLayer(void);
char (*rssi_get_f(void))(char);
void rssi_set_f( char (*f) (char));
#endif


