// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

/* -- BEGIN LICENSE BLOCK ----------------------------------------------

 Copyright (c) 2013, TB
 All rights reserved.

 Redistribution and use in source and binary forms are permitted
 provided that the above copyright notice and this paragraph are
 duplicated in all such forms and that any documentation,
 advertising materials, and other materials related to such
 distribution and use acknowledge that the software was developed
 by TB. The name of the
 TB may not be used to endorse or promote products derived
 from this software without specific prior written permission.
 THIS SOFTWARE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.

 -- END LICENSE BLOCK ----------------------------------------------*/

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Tobias Bär <baer@fzi.de> Jan Aidel <aiden@fzi.de>
 * \date    2013-01-11
 *
 */
//----------------------------------------------------------------------

#include <marble/MarbleWidget.h>
#include <marble/MarbleModel.h>
#include <marble/MapThemeManager.h>
#include <marble/GeoPainter.h>
#include <marble/GeoDataCoordinates.h>
#include <marble/Route.h>
#include <marble/RoutingModel.h>
// Qt Includes
#include <QLineEdit>
#include <QFileInfo>
#include <QFileDialog>
#include <QStringList>
#include <QStandardItemModel>
#include <QModelIndex>

#include <pluginlib/class_list_macros.h>
#include <ros/package.h>

#include "rqt_marble/marble_plugin.h"
#include "rqt_marble/bridge_ros_marble.h"

// @TODO: setDistance does not work on reloading
// @TODO: ComboBox for the MarbleWidget projection method
// @TOOD: Draw icon on the current gps pos (MarbleWidget needs to be subclassed (custom paint))

namespace rqt_marble
{

MarblePlugin::MarblePlugin() :
    rqt_gui_cpp::Plugin(), widget_(0)
{
  // give QObjects reasonable names
  setObjectName("MarbleWidgetPlugin");

  //Initialize variable as NULL so that we can check if the expected instance
  //is created or not by something like
  //  if this->ros_navigation == NULL
  this->ros_navigation = NULL;
}

void MarblePlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
  // access standalone command line arguments
  QStringList argv = context.argv();

  // create QWidget
  widget_ = new QWidget();

  // add widget to the user interface
  ui_.setupUi(widget_);
  ui_.MarbleWidget->setMapThemeId("earth/openstreetmap/openstreetmap.dgml");
  ui_.MarbleWidget->setProjection(Marble::Mercator);
  //ui_.MarbleWidget->centerOn(115.87164, -31.93452, false); // My Happy Place: The Scotto
  ui_.MarbleWidget->centerOn(-122.0795, 37.4000, false); // OSRF
  ui_.MarbleWidget->setDistance(0.05);

  context.addWidget(widget_);
  ui_.comboBox_theme->setModel(ui_.MarbleWidget->model()->mapThemeManager()->mapThemeModel());

  //set refresh icon
  QIcon refresh_icon;
  std::string path = ros::package::getPath("rqt_marble") + "/etc/refresh.png";
  QString icon_path(path.c_str());
  refresh_icon.addFile(icon_path);
  ui_.refreshButton->setIcon(refresh_icon);

  FindGPSTopics();

  // Connections
  connect(ui_.comboBox, SIGNAL(activated (const QString &)), this, SLOT (ChangeGPSTopic(const QString &)));
  connect(ui_.refreshButton, SIGNAL(clicked()), this, SLOT(FindGPSTopics()));

  connect(this, SIGNAL(NewGPSPosition(qreal, qreal)), ui_.MarbleWidget, SLOT(centerOn(qreal, qreal)));
//  connect( ui_.lineEdit_topic , SIGNAL(editingFinished()) , this , SLOT( ChangeGPSTopic()) );
  connect(ui_.lineEdit_kml, SIGNAL(returnPressed()), this, SLOT(SetKMLFile()));
  connect(ui_.comboBox_theme, SIGNAL(currentIndexChanged(int)), this, SLOT(ChangeMarbleModelTheme(int)));
  connect(ui_._checkbox_navigation, SIGNAL(clicked(bool)), this, SLOT(enableNavigation(bool)));

// AutoNavigation Connections ... soon
//  m_autoNavigation = new Marble::AutoNavigation(ui_.MarbleWidget->model(), ui_.MarbleWidget->viewport(), this);
//
//  connect(m_autoNavigation, SIGNAL( zoomIn( FlyToMode ) ), ui_.MarbleWidget, SLOT( zoomIn() ));
//  connect(m_autoNavigation, SIGNAL( zoomOut( FlyToMode ) ), ui_.MarbleWidget, SLOT( zoomOut() ));
//  connect(m_autoNavigation, SIGNAL( centerOn( const GeoDataCoordinates &, bool ) ), ui_.MarbleWidget,
//          SLOT( centerOn( const GeoDataCoordinates & ) ));
//  connect(ui_.MarbleWidget, SIGNAL( visibleLatLonAltBoxChanged() ), m_autoNavigation, SLOT( inhibitAutoAdjustments() ));

  //130s test
  manager = ui_.MarbleWidget->model()->routingManager();
  //this->request = manager->routeRequest();
  this->routeModel = manager->routingModel();
  connect(this->routeModel, SIGNAL(currentRouteChanged()), this, SLOT(routeChanged()));
}

void MarblePlugin::FindGPSTopics()
{
  using namespace ros::master;
  std::vector<TopicInfo> topic_infos;
  getTopics(topic_infos);

  ui_.comboBox->clear();
  for (std::vector<TopicInfo>::iterator it = topic_infos.begin(); it != topic_infos.end(); it++)
  {
    TopicInfo topic = (TopicInfo)(*it);
    if (topic.datatype.compare("sensor_msgs/NavSatFix") == 0)
    {
//            std::cout << "found " << topic.name << std::endl;
      QString lineEdit_string(topic.name.c_str());
      ui_.comboBox->addItem(lineEdit_string);
    }
  }
}

void MarblePlugin::shutdownPlugin()
{
// unregister all publishers here
  m_sat_nav_fix_subscriber.shutdown();
}

void MarblePlugin::ChangeMarbleModelTheme(int idx)
{
  QStandardItemModel* model = ui_.MarbleWidget->model()->mapThemeManager()->mapThemeModel();
  QModelIndex index = model->index(idx, 0);
  QString theme = model->data(index, Qt::UserRole + 1).toString();

  ui_.MarbleWidget->setMapThemeId(theme);
}

void MarblePlugin::ChangeGPSTopic(const QString &topic_name)
{
  m_sat_nav_fix_subscriber.shutdown();
  m_sat_nav_fix_subscriber = getNodeHandle().subscribe<sensor_msgs::NavSatFix>(topic_name.toStdString().c_str(), 10,
                                                                               &MarblePlugin::GpsCallback, this);
}

void MarblePlugin::SetKMLFile(bool envoke_file_dialog)
{
  QFileInfo fi(ui_.lineEdit_kml->text());

  if (!fi.isFile() && envoke_file_dialog)
  {
    QString fn = QFileDialog::getOpenFileName(0, tr("Open Geo Data File"), tr(""), tr("Geo Data Files (*.kml)"));
    fi.setFile(fn);
  }

  if (fi.isFile())
  {
    ui_.MarbleWidget->model()->addGeoDataFile(fi.absoluteFilePath());

    ui_.lineEdit_kml->setText(fi.absoluteFilePath());
  }
  else
  {
    ui_.lineEdit_kml->setText("");
  }
}

void MarblePlugin::GpsCallback(const sensor_msgs::NavSatFixConstPtr& gpspt)
{
// std::cout << "GPS Callback " << gpspt->longitude << " " << gpspt->latitude << std::endl;
  assert(widget_);

// Emit NewGPSPosition only, if it changes significantly. Has to be somehow related to the zoom
  static qreal _x = -1;
  static qreal _y = -1;

  qreal x;
  qreal y;

// Recenter if lat long is not on screen
  bool recenter = !ui_.MarbleWidget->screenCoordinates(gpspt->longitude, gpspt->latitude, x, y);
  recenter |= ui_.checkBox_center->isChecked();

// Recenter if lat long within <threshold> pixels away from center
  qreal threshold = 20;
  recenter |= ((x - _x) * (x - _x) + (y - _y) * (y - _y)) > threshold;

  if (recenter)
  {
    emit NewGPSPosition(gpspt->longitude, gpspt->latitude);
    ui_.MarbleWidget->screenCoordinates(gpspt->longitude, gpspt->latitude, _x, _y);
  }

//130s testing
//  std::cout << "size of reqs=" << this->request->size() << std::endl;
//  for (int i = 0; i < this->request->size(); i++)
//  {
//    Marble::GeoDataCoordinates coord = this->request->at(i);
//    //std::cout << "req #" << i << " longitude=" << coord.lonToString().toStdString() << std::endl;
//  }

//  Marble::RoutingModel* rmodel = this->manager->routingModel();
//  Marble::Route route = rmodel->route();
//  ROS_INFO("size of route =", route.size());
//  for (int i = 0; i < route.size(); i++)
//  {
//    Marble::GeoDataLineString route_segment_line_str = route.at(i).path();
//    for (int j = 0; j < route_segment_line_str.size(); j++)
//    {
//      Marble::GeoDataCoordinates coord = route_segment_line_str.at(j);
//      std::cout << i << "#seg" << " coord#" << j << " longi=" << coord.longitude() << " lat=" << coord.latitude()
//          << std::endl;
//    }
//  }
}

void MarblePlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
// save intrinsic configuration, usually using:
  QString topic(m_sat_nav_fix_subscriber.getTopic().c_str());
  instance_settings.setValue("rqt_marble_topic", topic);
  instance_settings.setValue("rqt_marble_kml_file", ui_.lineEdit_kml->text().replace(".", "___dot_replacement___"));
  instance_settings.setValue("rqt_marble_zoom", ui_.MarbleWidget->distance());
  instance_settings.setValue("marble_theme_index", ui_.comboBox_theme->currentIndex());
  instance_settings.setValue("marble_center", ui_.checkBox_center->isChecked());
}

void MarblePlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
                                   const qt_gui_cpp::Settings& instance_settings)
{
// restore intrinsic configuration, usually using:
  const QString topic = instance_settings.value("rqt_marble_topic").toString();
  ChangeGPSTopic(topic);

  ui_.lineEdit_kml->setText(
      instance_settings.value("rqt_marble_kml_file", "").toString().replace("___dot_replacement___", "."));
  ui_.comboBox_theme->setCurrentIndex(instance_settings.value("marble_theme_index", 0).toInt());
  ui_.checkBox_center->setChecked(instance_settings.value("marble_center", true).toBool());

// std::cout << "Set distance " << instance_settings.value( "rqt_marble_zoom" ).toReal() << std::endl;

  SetKMLFile(false);

// @TODO: Does not work since the KML loading changes the zoom
  ui_.MarbleWidget->setDistance(instance_settings.value("rqt_marble_zoom", 0.05).toReal());
}

/*bool hasConfiguration() const
 {
 return true;
 }

 void triggerConfiguration()
 {
 // Usually used to open a dialog to offer the user a set of configuration
 }*/

void MarblePlugin::enableNavigation(bool checked)
{
  if (checked)
  {
    this->ros_navigation = new rqt_marble::BridgeRosMarble();
    this->ros_navigation->setDoNavigation(true);
  }
  else
  {
    this->ros_navigation->setDoNavigation(false);
  }
}

void MarblePlugin::routeChanged()
{
  ROS_INFO("MarblePlugin routeChanged 1");

  //TODO: Check if this->ros_navigation is instantiated. Checking if the memory
  // space is not null is NOT enough (causes segfault).

  if (this->ros_navigation == NULL)
  {
    //TODO: raise exception
    this->ros_navigation = new rqt_marble::BridgeRosMarble();
    ROS_INFO("MarblePlugin routeChanged 2");
  }
  ROS_INFO("MarblePlugin routeChanged 3");
  Marble::Route route = this->routeModel->route();
  this->ros_navigation->publishRouteInGps(route);
}

} // namespace

PLUGINLIB_EXPORT_CLASS(rqt_marble::MarblePlugin, rqt_gui_cpp::Plugin)
