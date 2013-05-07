/**
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2013, Open Source Robotics Foundation (OSRF).
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 **/
// Author: Isaac Saito

#ifndef _MARBLE_ROUTING_H
#define _MARBLE_ROUTING_H

#include <marble/Route.h>
#include <QMutex>
#include <QtCore/QObject>

#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>
#include <sensor_msgs/NavSatFix.h>

#include <rqt_cpp_common/RouteGps.h>
#include <ui_marble_plugin.h>

namespace rqt_marble
{

/**
 * BridgeRosMarble object is initially intended to work as a bridge between
 * ROS & Marble so that MarblePlugin can be decoupled from ROS & Marble as
 * much as possible.
 *
 * However, for the coding simplicity, MarblePlugin by design involves ROS & Marble
 * tasks.
 */
class BridgeRosMarble : public rqt_gui_cpp::Plugin
{
  Q_OBJECT

public:

  BridgeRosMarble();
  void getGpsCoord();
  /**
   * @arg Route route: this object consists of segments of a route.
   *                   Each segment consists of GPS coordinates.
   * @return: when either do_navigation is False or the size of the argument
   * route is 0, this function just returns.
   */
  rqt_cpp_common::RouteGps publishRouteInGps(Marble::Route route);
  void setDoNavigation(bool doNav);
  //void GpsCallback(const sensor_msgs::NavSatFixConstPtr& gpspt);

private:
  ros::NodeHandle _node;
  ros::Publisher _publisher;
  bool do_navigation;
};
} // namespace
#endif // _MARBLE_ROUTING_H
