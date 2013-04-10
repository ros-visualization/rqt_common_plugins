#ifndef _MARBLE_ROUTING_H
#define _MARBLE_ROUTING_H

#include <marble/Route.h>
#include <QMutex>
#include <QtCore/QObject>

#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>
#include <sensor_msgs/NavSatFix.h>

#include <ui_marble_plugin.h>

namespace rqt_marble
{

class BridgeRosMarble : public rqt_gui_cpp::Plugin
{
  Q_OBJECT

public:

  BridgeRosMarble();
  void setDoNavigation(bool doNav);
  void publishRouteInGps(Marble::Route route);
  void getGpsCoord();
  //void GpsCallback(const sensor_msgs::NavSatFixConstPtr& gpspt);

private:
  ros::NodeHandle _node;
  ros::Publisher _publisher;
  bool do_navigation;
};
} // namespace
#endif // _MARBLE_ROUTING_H
