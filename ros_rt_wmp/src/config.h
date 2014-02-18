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
#include <std_msgs/Float64.h>
#include <roscpp_tutorials/TwoInts.h>
#include <theora_image_transport/Packet.h>
#include <audio_common_msgs/AudioData.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>

/* INCLUDE TOPIC and SERVICES HERE */
BEGIN_TOPIC_DEFINITION

//QOS_TOPIC("video/theora", theora_image_transport::Packet, "1", "0", 1, 5, 250000);
//QOS_TOPIC("scan", sensor_msgs::LaserScan, "1", "0", 5,10,108000);

TOPIC("video/theora", theora_image_transport::Packet, "1", "0", 1);
TOPIC("audio", audio_common_msgs::AudioData, "1", "0", 2);
TOPIC("scan", sensor_msgs::LaserScan, "1", "0", 5);
TOPIC("cmd_vel", geometry_msgs::Twist, "0", "1", 10);

TOPIC_TF("0","1", 3);

DECIMATE_TOPIC("image_raw", sensor_msgs::Image, "1", 2);

END_TOPIC_DEFINITION

#endif
