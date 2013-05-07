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

#include <rqt_cpp_common/widgets_list.h>

using namespace rqt_cpp_common;

WidgetsList::WidgetsList()
{
  this->ui_._vlayout_for_widgets = new QVBoxLayout(
      ui_._scrollarea_holder_widget);
}

/**
 * Add the given pair of id_widget and widget into the internal table that
 * records the existing widgets. Then add the widget to the layout.
 *
 * @param widget: This QWidget instance MUST have a "parent" pointing to
 *                WidgetsList. This "parent" value will be used in other
 *                functions (eg. WidgetsList::remove_node).
 */
void WidgetsList::add_widget(QString id_widget, QWidget *widget)
{
  if (this->table_widgets->contains(id_widget))
  { //Do nothiing
  }
  else
  {
    this->table_widgets->insert(id_widget, widget);
    this->ui_._vlayout_for_widgets->addWidget(widget);
  }
}

/**
 * @exception ros::Exception when the given widget_id doesn't exist or no item
 *            is associated.
 */
void WidgetsList::remove_node(QString widget_id)
{
  QWidget widget_tobe_removed;
  QWidget *tobe_returned_when_correspondent_notfound;
  //widget_tobe_removed = this->table_widgets->value(widget_id);
  if (this->table_widgets->contains(widget_id))
  {
    ros::Exception(
        "The given widget_id = " + widget_id.toStdString()
            + " doesn't exist or no item is associated.");
  }
  this->ui_._vlayout_for_widgets->removeWidget(&widget_tobe_removed);
  this->table_widgets->take(widget_id);
}
