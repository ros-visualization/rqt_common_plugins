#ifndef _MARBLE_ROUTING_H
#define _MARBLE_ROUTING_H

#include <marble/Route.h>
#include <QMutex>
#include <QtCore/QObject>

#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>
#include <sensor_msgs/NavSatFix.h>

#include <rqt_marble/RouteGps.h>
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
  rqt_marble::RouteGps publishRouteInGps(Marble::Route route);
  void setDoNavigation(bool doNav);
  //void GpsCallback(const sensor_msgs::NavSatFixConstPtr& gpspt);

private:
  ros::NodeHandle _node;
  ros::Publisher _publisher;
  bool do_navigation;
};
} // namespace
#endif // _MARBLE_ROUTING_H
