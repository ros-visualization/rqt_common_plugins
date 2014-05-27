#!/usr/bin/env python

# Copyright (c) 2014, Austin Hendrix
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


from qt_gui_py_common.simple_settings_dialog import SimpleSettingsDialog
from python_qt_binding.QtGui import QWidget, QHBoxLayout

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


class DataPlot(QWidget):
    """A widget for displaying a plot of data

    The DataPlot widget displays a plot, on one of several plotting backends,
    depending on which backend(s) are available at runtime. It currently 
    supports PyQtGraph, MatPlot and QwtPlot backends.

    The DataPlot widget manages the plot backend internally, and can save
    and restore the internal state using `save_settings` and `restore_settings`
    functions.

    Currently, the user MUST call `restore_settings` before using the widget,
    to cause the creation of the enclosed plotting widget.
    """
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

    def __init__(self, parent=None):
        """Create a new, empty DataPlot

        This will raise a RuntimeError if none of the supported plotting
        backends can be found
        """
        super(DataPlot, self).__init__(parent)
        self._plot_index = 0
        self._autoscroll = True

        # the backend widget that we're trying to hide/abstract
        self._data_plot_widget = None
        self._curves = {}

        self._layout = QHBoxLayout()
        self.setLayout(self._layout)

        enabled_plot_types = [pt for pt in self.plot_types if pt['enabled']]
        if not enabled_plot_types:
            version_info = ' and PySide > 1.1.0' if QT_BINDING == 'pyside' else ''
            raise RuntimeError('No usable plot type found. Install at least one of: PyQtGraph, MatPlotLib (at least 1.1.0%s) or Python-Qwt5.' % version_info)

        self.show()

    def _switch_data_plot_widget(self, plot_index):
        """Internal method for activating a plotting backend by index"""
        # check if selected plot type is available
        if not self.plot_types[plot_index]['enabled']:
            # find other available plot type
            for index, plot_type in enumerate(self.plot_types):
                if plot_type['enabled']:
                    plot_index = index
                    break

        self._plot_index = plot_index
        selected_plot = self.plot_types[plot_index]

        print "Creating new plot widget: %s" % ( self.getTitle() )

        if self._data_plot_widget:
            self._layout.removeWidget(self._data_plot_widget)
            self._data_plot_widget.close()
            self._data_plot_widget = None

        self._data_plot_widget = selected_plot['widget_class'](self)
        self._data_plot_widget.autoscroll(self._autoscroll)
        self._layout.addWidget(self._data_plot_widget)

        # restore old data
        for curve_id in self._curves:
            curve = self._curves[curve_id]
            self._data_plot_widget.add_curve(curve_id, curve['name'],
                    curve['x'], curve['y'])
        self._data_plot_widget.redraw()

    # interface out to the managing GUI component: get title, save, restore, 
    # etc
    def getTitle(self):
        """get the title of the current plotting backend"""
        return self.plot_types[self._plot_index]['title']

    def save_settings(self, plugin_settings, instance_settings):
        """Save the settings associated with this widget

        Currently, this is just the plot type, but may include more useful
        data in the future"""
        instance_settings.set_value('plot_type', self._plot_index)

    def restore_settings(self, plugin_settings, instance_settings):
        """Restore the settings for this widget

        Currently, this just restores the plot type."""
        self._switch_data_plot_widget(int(instance_settings.value('plot_type', 0)))

    def doSettingsDialog(self):
        """Present the user with a dialog for choosing the plot backend

        This displays a SimpleSettingsDialog asking the user to choose a
        plot type, gets the result, and updates the plot type as necessary
        
        This method is blocking"""
        dialog = SimpleSettingsDialog(title='Plot Options')
        dialog.add_exclusive_option_group(title='Plot Type', options=self.plot_types, selected_index=self._plot_index)
        plot_type = dialog.get_settings()[0]
        if plot_type is not None and plot_type['selected_index'] is not None and self._plot_index != plot_type['selected_index']:
            self._switch_data_plot_widget(plot_type['selected_index'])

    # interface out to the managing DATA component: load data, update data,
    # etc
    def autoscroll(self, enabled=True):
        """Enable or disable autoscrolling of the plot"""
        self._autoscroll = enabled
        if self._data_plot_widget:
            self._data_plot_widget.autoscroll(enabled)

    def redraw(self):
        """Redraw the underlying plot

        This causes the underlying plot to be redrawn. This is usually used
        after adding or updating the plot data"""
        if self._data_plot_widget:
            self._data_plot_widget.redraw()

    def add_curve(self, curve_id, curve_name, data_x, data_y):
        """Add a new, named curve to this plot

        Add a curve named `curve_name` to the plot, with initial data series
        `data_x` and `data_y`.
        
        Future references to this curve should use the provided `curve_id`

        Note that the plot is not redraw automatically; call `redraw()` to make
        any changes visible to the user.
        """
        self._curves[curve_id] = { 'x': data_x, 'y': data_y, 'name': curve_name }
        if self._data_plot_widget:
            self._data_plot_widget.add_curve(curve_id, curve_name, data_x, data_y)

    def remove_curve(self, curve_id):
        """Remove the specified curve from this plot"""
        if curve_id in self._curves:
            del self._curves[curve_id]
        if self._data_plot_widget:
            self._data_plot_widget.remove_curve(curve_id)

    def update_values(self, curve_id, values_x, values_y):
        """Append new data to an existing curve
        
        `values_x` and `values_y` will be appended to the existing data for
        `curve_id`

        Note that the plot is not redraw automatically; call `redraw()` to make
        any changes visible to the user.
        """
        curve = self._curves[curve_id]
        if curve:
            curve['x'].extend(values_x)
            curve['y'].extend(values_y)
        if self._data_plot_widget:
            self._data_plot_widget.update_values(curve_id, values_x, values_y)

    def clear_values(self, curve_id=None):
        """Clear the values for the specified curve, or all curves

        This will erase the data series associaed with `curve_id`, or all
        curves if `curve_id` is not present or is None

        Note that the plot is not redraw automatically; call `redraw()` to make
        any changes visible to the user.
        """
        pass
