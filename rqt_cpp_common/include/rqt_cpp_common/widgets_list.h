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
#ifndef _WIDGETS_LIST_H
#define _WIDGETS_LIST_H

#include <QHash>
#include <QList>
#include <QString>
#include <QVBoxLayout>
#include <QWidget>

#include <ros/ros.h>

#include <ui_widgets_list.h> // Generated under %CATKIN_WS%/build/%prj% by Catkin.
namespace rqt_cpp_common
{

/**
 * Porting similar functionality of rqt_py_common.paramedit_widget to C++.
 *
 * This class represents a pane where parameter editor widgets of multiple
 * nodes are shown. Example is found in rqt_reconfigure (although it's in
 * python); this pane occupies right half of the entire visible area.
 *
 * TODO: As of Apr 2013, this class can put the widgets only on virtical
 *       layout. Need to improve so that user can choose horizontal layout.
 *
 * @author: Isaac Saito
 */
class WidgetsList : public QWidget
{
  Q_OBJECT

public:

  WidgetsList();

private:
  QHash<QString, QWidget*> *table_widgets;
  Ui::WidgetsList ui_;

  void add_widget(QString id_widget, QWidget *widget);
  void remove_node(QString widget_id);
};
} // namespace
#endif // _WIDGETS_LIST_H
