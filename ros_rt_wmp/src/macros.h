/*------------------------------------------------------------------------
 *---------------------           ros_rt_wmp              --------------------
 *------------------------------------------------------------------------
 *                                                         V0.1B  15/09/11
 *
 *
 *  File: ./src/macros.h
 *  Authors: Danilo Tardioli
 *  ----------------------------------------------------------------------
 *  Copyright (C) 2000-2011, Universidad de Zaragoza, SPAIN
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
#ifndef MACROS_H_
#define MACROS_H_

#include "TopicManager.h"
#include "ServiceManager.h"
#include "TFManager.h"
#include "WMPTopicManager.h"
#include "WMPServiceManager.h"
#include "WMPNodeManager.h"

#define TOPIC(name,type,src,dest, prio)          m = new TopicManager<type> (n, port, std::string(name), src, dest, prio, false); \
								                 w->addManager(name, m); port++;


#define QOS_TOPIC(name,type,src,dest, prio, queue_size, period)          m = new TopicManager<type> (n, port, std::string(name), src, dest, prio, false, queue_size, period); \
								                 w->addManager(name, m); port++;

#define TOPIC_START_STOPPED(name,type,src,dest, prio)     m = new TopicManager<type> (n, port, std::string(name), src, dest, prio, false); \
								                 w->addManager(name, m); port++; m->stop();

#define DYNAMIC_TOPIC(name,type,src, prio)       m = new TopicManager<type> (n, port, std::string(name), src, prio, false); \
                                                 w->addManager(name, m); port++;

#define WMP_TOPIC(name,type,src)                 m = new WMPTopicManager<type> (n, port, std::string(name), src, false); \
						                         w->addManager(name, m); port++;

#define SERVICE(name,type,host, prio)            m = new ServiceManager<type> (n, port, std::string(name),host, prio); \
								                 w->addManager(name, m); port+=2;

#define DYNAMIC_SERVICE(name,type, prio)         m = new ServiceManager<type> (n, port, std::string(name), prio); \
                                                 w->addManager(name, m); port+=2;

#define WMP_SERVICE(name,type)                   m = new WMPServiceManager<type> (n, port, std::string(name)); \
								                 w->addManager(name, m); port+=2;

#define TOPIC_TF(src,dest)            			 m = new TFManager(n, port, src, dest, false) ; \
						                         w->addManager("/tf", m); port++;

#define DYNAMIC_TOPIC_TF(src)         			 m = new TFManager(n, port, src, false) ; \
						                         w->addManager("/tf", m); port++;

#define SHAPESHIFTER(name,src,dest)              m = new ShapeShifterManager (n, port, std::string(name), src, dest,false); \
								                 w->addManager(name, m); port++;


#define DECIMATE_TOPIC(name, type, src, decimation)          m = new TopicManager<type> (n, port, std::string(name), src, "", 0, false); \
								                 m->setDecimation(decimation); w->addManager(name, m); port++;

#define BEGIN_TOPIC_DEFINITION                   void define_objects(ros::NodeHandle * n, WMPNodeManager * w){ Manager * m; int port = 4;

#define END_TOPIC_DEFINITION }

#endif /* MACROS_H_ */
