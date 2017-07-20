/*------------------------------------------------------------------------
 *---------------------           ros_rt_wmp              --------------------
 *------------------------------------------------------------------------
 *                                                         V0.1B  15/09/11
 *
 *
 *  File: ./src/config.h
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

#ifndef CONFIG_H_
#define CONFIG_H_
#include "macros.h"

/* INCLUDE DATA TYPE HERE */

#include <std_msgs/Int8.h>
#include <theora_image_transport/Packet.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>
#include <mavros_msgs/BatteryStatus.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/State.h>

/* INCLUDE TOPIC and SERVICES HERE */
BEGIN_TOPIC_DEFINITION

PTOPIC(theora_image_transport::Packet, "/net/image/theora",                 "0", "1", 51, 1,   500);
PTOPIC(sensor_msgs::LaserScan,         "/scan/shaped",                      "0", "1", 50, 200, 500);
PTOPIC(geometry_msgs::PoseStamped,     "/mavros/local_position/pose",       "0", "1", 50, 100, 500);
PTOPIC(geometry_msgs::TwistStamped,    "/mavros/setpoint_velocity/cmd_vel", "0", "1", 50, 100, 500);
PTOPIC(nav_msgs::Path,                 "/mavtest/local_plan",               "0", "1", 50, 200, 500);
PTOPIC(nav_msgs::Odometry,             "/rtabmap/odom",                     "0", "1", 50, 200, 500);
PTOPIC(mavros_msgs::State,             "/mavros/state",                     "0", "1", 55, 500, 500);
PTOPIC(mavros_msgs::BatteryStatus,     "/mavros/battery",                   "0", "1", 56, 100, 500);
PTOPIC(std_msgs::Int8,                 "/int8",                             "0", "1", 50, 100, 500);

END_TOPIC_DEFINITION

#endif
