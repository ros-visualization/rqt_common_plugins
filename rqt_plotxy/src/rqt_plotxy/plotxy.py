#!/usr/bin/env python

# Copyright (c) 2011, Dorian Scholz, TU Darmstadt
# Copyright (c) 2016, Rafael Bailon-Ruiz, LAAS-CNRS
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
#   * Neither the name of the TU Darmstadt nor LAAS-CNRS nor the names
#     of its contributors may be used to endorse or promote products
#     derived from this software without specific prior written permission.
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

import argparse

from python_qt_binding import QT_BINDING
from python_qt_binding.QtCore import qDebug
from rqt_gui_py.plugin import Plugin

from rqt_py_common.ini_helper import pack, unpack
from rqt_plot.data_plot import DataPlot

from .plot_widget_xy import PlotWidgetXY


class PlotXY(Plugin):

    def __init__(self, context):
        super(PlotXY, self).__init__(context)
        self.setObjectName('PlotXY')

        self._context = context

        parser = argparse.ArgumentParser(prog='rqt_plotxy', add_help=False)
        PlotXY.add_arguments(parser)
        self._args = parser.parse_args(context.argv())

        self._widget = PlotWidgetXY(start_paused=self._args.start_paused)
        self._data_plot = DataPlot(self._widget)

        # disable autoscaling of X, and set a sane default range
        self._data_plot.set_autoscale(x=False)
        self._data_plot.set_autoscale(y=DataPlot.SCALE_EXTEND|DataPlot.SCALE_VISIBLE)
        self._data_plot.set_xlim([0, 10.0])

        self._widget.switch_data_plot_widget(self._data_plot)
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

    @staticmethod
    def add_arguments(parser):
        group = parser.add_argument_group('Options for rqt_plotxy plugin')
        group.add_argument('-P', '--pause', action='store_true', dest='start_paused',
            help='Start in paused state')
        group.add_argument('-e', '--empty', action='store_true', dest='start_empty',
            help='Start without restoring previous topics')

    def _update_title(self):
        self._widget.setWindowTitle(self._data_plot.getTitle())
        if self._context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % self._context.serial_number()))

    def save_settings(self, plugin_settings, instance_settings):
        self._data_plot.save_settings(plugin_settings, instance_settings)
        instance_settings.set_value('autoscroll', self._widget.autoscroll_checkbox.isChecked())
        topics = self._widget._rosdata.values()
        topic_names_x = [t.ros_data_x.name for t in topics]
        instance_settings.set_value('topicsX', pack(topic_names_x))
        topic_names_y = [t.ros_data_y.name for t in topics]
        instance_settings.set_value('topicsY', pack(topic_names_y))

    def restore_settings(self, plugin_settings, instance_settings):
        autoscroll = instance_settings.value('autoscroll', True) in [True, 'true']
        self._widget.autoscroll_checkbox.setChecked(autoscroll)
        self._data_plot.autoscroll(autoscroll)

        self._update_title()

        if len(self._widget._rosdata.keys()) == 0 and not self._args.start_empty:
            topicsX = unpack(instance_settings.value('topicsX', []))
            topicsY = unpack(instance_settings.value('topicsY', []))
            if topicsX and topicsY and len(topicsX) == len(topicsY):
                for topic in zip(topicsX, topicsY):
                    self._widget.add_topic(*topic)

        self._data_plot.restore_settings(plugin_settings, instance_settings)

    def trigger_configuration(self):
        self._data_plot.doSettingsDialog()
        self._update_title()

    def shutdown_plugin(self):
        self._widget.clean_up_subscribers()
