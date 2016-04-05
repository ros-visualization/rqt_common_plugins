#!/usr/bin/env python

# Copyright (c) 2011, Dorian Scholz, TU Darmstadt
# Copyright (c) 2016, Rafael Bailon-Ruiz
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the TU Darmstadt nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from python_qt_binding.QtCore import qWarning, Slot
from python_qt_binding.QtGui import QLabel, QLineEdit

import rospy

from rqt_py_common.topic_completer import TopicCompleter

from rqt_plot.rosplot import RosPlotException
from rqt_plot.plot_widget import get_plot_fields, is_plottable, PlotWidget

from . rosplotxy import ROSDataXY


class PlotWidgetXY(PlotWidget):

    def __init__(self, initial_topics=None, start_paused=False):
        super(PlotWidgetXY, self).__init__(
            initial_topics=initial_topics, start_paused=start_paused)

        self.setObjectName('PlotWidgetXY')

        self.label.setText("Topic Y")

        self.topic_edit_x = QLineEdit()
        self.topic_edit_x.setText("/")
        self.dataPlotControls.insertWidget(0, self.topic_edit_x)

        self.labelX = QLabel("Topic X")
        self.dataPlotControls.insertWidget(0, self.labelX)

        self._topic_completer_x = TopicCompleter(self.topic_edit_x)
        self._topic_completer_x.update_topics()
        self.topic_edit_x.setCompleter(self._topic_completer_x)

    @Slot('QDragEnterEvent*')
    def dragEnterEvent(self, event):
        pass  # No drag'n drop for now

    @Slot('QDropEvent*')
    def dropEvent(self, event):
        pass  # No drag'n drop for now

    @Slot(str)
    def on_topic_edit_textChanged(self, topic_name):
        # on empty topic name, update topics
        if topic_name in ('', '/'):
            self._topic_completer.update_topics()

        plottable, message = is_plottable(topic_name)
        plottable_x, message_x = is_plottable(self.topic_edit_x.text())
        self.subscribe_topic_button.setEnabled(plottable and plottable_x)
        self.subscribe_topic_button.setToolTip(
            "Topic Y: {}\nTopic X: {}".format(message, message_x))

    @Slot(str)
    def on_topic_edit_x_textChanged(self, topic_name):
        # on empty topic name, update topics
        if topic_name in ('', '/'):
            self._topic_completer_x.update_topics()

        plottable, message = is_plottable(self.topic_edit.text())
        plottable_x, message_x = is_plottable(topic_name)
        self.subscribe_topic_button.setEnabled(plottable and plottable_x)
        self.subscribe_topic_button.setToolTip(
            "Topic Y: {}\nTopic X: {}".format(message, message_x))

    @Slot()
    def on_topic_edit_returnPressed(self):
        if self.subscribe_topic_button.isEnabled():
            self.add_topic(str(self.topic_edit_x.text()),
                           str(self.topic_edit.text()))

    @Slot()
    def on_subscribe_topic_button_clicked(self):
        self.add_topic(str(self.topic_edit_x.text()),
                       str(self.topic_edit.text()))

    def update_plot(self):
        if self.data_plot is not None:
            needs_redraw = False
            for topic_name, rosdata in self._rosdata.items():
                try:
                    data_x, data_y = rosdata.next()
                    if data_x or data_y:
                        self.data_plot.update_values(topic_name, data_x,
                                                     data_y, sort_data=False)
                        needs_redraw = True
                except RosPlotException as e:
                    qWarning('PlotWidget.update_plot(): ' +
                             'error in rosplot: {}'.format(e))
            if needs_redraw:
                self.data_plot.redraw()

    def add_topic(self, topic_name_x, topic_name_y):
        topics_changed = False
        if len(get_plot_fields(topic_name_x)[0]) != \
           len(get_plot_fields(topic_name_y)[0]):
            return
        for name_x, name_y in zip(get_plot_fields(topic_name_x)[0],
                                  get_plot_fields(topic_name_y)[0]):
            if name_y in self._rosdata:
                qWarning('PlotWidget.add_topic(): ' +
                         'topic already subscribed: {}'.format(name_y))
                continue
            self._rosdata[name_y] = ROSDataXY(
                (name_x, name_y), self._start_time)
            if self._rosdata[name_y].error is not None:
                qWarning(str(self._rosdata[name_y].error))
                del self._rosdata[name_y]
            else:
                data_x, data_y = self._rosdata[name_y].next()
                self.data_plot.add_curve(name_y, "{}\n{}".format(
                    name_y, name_x), data_x, data_y)
                topics_changed = True

        if topics_changed:
            self._subscribed_topics_changed()
