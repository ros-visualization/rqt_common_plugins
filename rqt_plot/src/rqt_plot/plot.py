#!/usr/bin/env python

# Copyright (c) 2011, Dorian Scholz, TU Darmstadt
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

import argparse

from python_qt_binding import QT_BINDING
from python_qt_binding.QtCore import qDebug
from qt_gui_py_common.simple_settings_dialog import SimpleSettingsDialog
import rospy
from rqt_gui_py.plugin import Plugin

from rqt_py_common.ini_helper import pack, unpack

from .plot_widget import PlotWidget

try:
    from pyqtgraph_data_plot import PyQtGraphDataPlot
except ImportError:
    qDebug('[DEBUG] rqt_plot.plot: import of PyQtGraphDataPlot failed (trying other backends)')
    PyQtGraphDataPlot = None

try:
    from mat_data_plot import MatDataPlot
except ImportError:
    qDebug('[DEBUG] rqt_plot.plot: import of MatDataPlot failed (trying other backends)')
    MatDataPlot = None

try:
    from qwt_data_plot import QwtDataPlot
except ImportError:
    qDebug('[DEBUG] rqt_plot.plot: import of QwtDataPlot failed (trying other backends)')
    QwtDataPlot = None


class Plot(Plugin):
    # plot types in order of priority
    plot_types = [
        {
            'title': 'PyQtGraph',
            'widget_class': PyQtGraphDataPlot,
            'description': 'Based on PyQtGraph\n- installer: http://luke.campagnola.me/code/pyqtgraph',
            'enabled': PyQtGraphDataPlot is not None,
        },
        {
            'title': 'MatPlot',
            'widget_class': MatDataPlot,
            'description': 'Based on MatPlotLib\n- needs most CPU\n- needs matplotlib >= 1.1.0\n- if using PySide: PySide > 1.1.0',
            'enabled': MatDataPlot is not None,
        },
        {
            'title': 'QwtPlot',
            'widget_class': QwtDataPlot,
            'description': 'Based on QwtPlot\n- does not use timestamps\n- uses least CPU\n- needs Python Qwt bindings',
            'enabled': QwtDataPlot is not None,
        },
    ]

    def __init__(self, context):
        super(Plot, self).__init__(context)
        self.setObjectName('Plot')

        enabled_plot_types = [pt for pt in self.plot_types if pt['enabled']]
        if not enabled_plot_types:
            version_info = ' and PySide > 1.1.0' if QT_BINDING == 'pyside' else ''
            raise RuntimeError('No usable plot type found. Install at least one of: PyQtGraph, MatPlotLib (at least 1.1.0%s) or Python-Qwt5.' % version_info)

        self._plot_type_index = 0
        self._context = context

        self._args = self._parse_args(context.argv())
        self._widget = PlotWidget(initial_topics=self._args.topics, start_paused=self._args.start_paused)
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

    def _parse_args(self, argv):
        parser = argparse.ArgumentParser(prog='rqt_plot', add_help=False)
        Plot.add_arguments(parser)
        args = parser.parse_args(argv)

        # convert topic arguments into topic names
        topic_list = []
        for t in args.topics:
            # c_topics is the list of topics to plot
            c_topics = []
            # compute combined topic list, t == '/foo/bar1,/baz/bar2'
            for sub_t in [x for x in t.split(',') if x]:
                # check for shorthand '/foo/field1:field2:field3'
                if ':' in sub_t:
                    base = sub_t[:sub_t.find(':')]
                    # the first prefix includes a field name, so save then strip it off
                    c_topics.append(base)
                    if not '/' in base:
                        parser.error("%s must contain a topic and field name" % sub_t)
                    base = base[:base.rfind('/')]

                    # compute the rest of the field names
                    fields = sub_t.split(':')[1:]
                    c_topics.extend(["%s/%s" % (base, f) for f in fields if f])
                else:
                    c_topics.append(sub_t)
            # #1053: resolve command-line topic names
            import rosgraph
            c_topics = [rosgraph.names.script_resolve_name('rqt_plot', n) for n in c_topics]
            if type(c_topics) == list:
                topic_list.extend(c_topics)
            else:
                topic_list.append(c_topics)
        args.topics = topic_list

        return args

    @staticmethod
    def add_arguments(parser):
        group = parser.add_argument_group('Options for rqt_plot plugin')
        group.add_argument('-P', '--pause', action='store_true', dest='start_paused',
            help='Start in paused state')
        group.add_argument('-e', '--empty', action='store_true', dest='start_empty',
            help='Start without restoring previous topics')
        group.add_argument('topics', nargs='*', default=[], help='Topics to plot')

    def _switch_data_plot_widget(self, plot_type_index):
        # check if selected plot type is available
        if not self.plot_types[plot_type_index]['enabled']:
            # find other available plot type
            for index, plot_type in enumerate(self.plot_types):
                if plot_type['enabled']:
                    plot_type_index = index
                    break

        self._plot_type_index = plot_type_index
        selected_plot = self.plot_types[plot_type_index]

        self._widget.switch_data_plot_widget(selected_plot['widget_class'](self._widget))
        self._widget.setWindowTitle(selected_plot['title'])
        if self._context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % self._context.serial_number()))

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value('plot_type', self._plot_type_index)
        instance_settings.set_value('topics', pack(self._widget._rosdata.keys()))
        instance_settings.set_value('show_legend', self._widget.togglemode())

    def restore_settings(self, plugin_settings, instance_settings):
        self._switch_data_plot_widget(int(instance_settings.value('plot_type', 0)))

        if len(self._widget._rosdata.keys()) == 0 and not self._args.start_empty:
            topics = unpack(instance_settings.value('topics', []))
            if topics:
                for topic in topics:
                    self._widget.add_topic(topic)
        _toggle = instance_settings.value('show_legend', 0)
        rospy.logdebug('restore_settings _toggle={}'.format(bool(_toggle)))
        self._widget.legend_toggled(_toggle in [True, 'true'])

    def trigger_configuration(self):
        dialog = SimpleSettingsDialog(title='Plot Options')
        dialog.add_exclusive_option_group(title='Plot Type', options=self.plot_types, selected_index=self._plot_type_index)
        plot_type = dialog.get_settings()[0]
        if plot_type is not None and plot_type['selected_index'] is not None and self._plot_type_index != plot_type['selected_index']:
            self._switch_data_plot_widget(plot_type['selected_index'])

    def shutdown_plugin(self):
        self._widget.clean_up_subscribers()
