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
#include <ros/exception.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>

#include <rqt_marble/bridge_ros_marble.h>

using namespace rqt_marble;
using namespace rqt_cpp_common;
using namespace sensor_msgs;

BridgeRosMarble::BridgeRosMarble()
{
  //ros::init(0, NULL, "BridgeLibMarble");
  this->do_navigation = true;
  this->_publisher = this->_node.advertise<RouteGps>("route_gps", 1000);
}

void BridgeRosMarble::setDoNavigation(bool doNav)
{
  this->do_navigation = doNav;
}

RouteGps BridgeRosMarble::publishRouteInGps(Marble::Route route)
{
  RouteGps route_gps;
  if (!this->do_navigation || route.size() == 0)
  {
    ros::Exception("Marble::Route passed does not contain route.");
  }

  int size_route = route.size();
  ROS_DEBUG("size of route =" + size_route);
  for (int i = 0; i < size_route; i++)
  {
    Marble::GeoDataLineString route_segment_line_str = route.at(i).path();
    for (int j = 0; j < route_segment_line_str.size(); j++)
    {
      Marble::GeoDataCoordinates coord = route_segment_line_str.at(j);
      // create GPS msg for ROS
      NavSatFix gps_msg = NavSatFix();
      gps_msg.latitude = coord.latitude();
      gps_msg.longitude = coord.longitude();
      gps_msg.status.status = NavSatStatus::STATUS_FIX;
      gps_msg.status.service = NavSatStatus::SERVICE_GPS;

      ROS_INFO("#%dth seg; coord#%dlongi=%f lat=%f", i, j, gps_msg.latitude,
               gps_msg.longitude);

      route_gps.routes.push_back(gps_msg);
    }
  }
  this->_publisher.publish(route_gps);
  return route_gps;
}
