#!/usr/bin/env python

# Copyright (c) 2011, Dorian Scholz
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

from python_qt_binding.QtCore import Slot, Qt, qWarning
from python_qt_binding.QtGui import QColor, QVBoxLayout, QWidget

from pyqtgraph import PlotWidget, mkPen
import numpy


class PyQtGraphDataPlot(QWidget):
    _colors = [Qt.red, Qt.blue, Qt.magenta, Qt.cyan, Qt.green, Qt.darkYellow, Qt.black, Qt.darkRed, Qt.gray, Qt.darkCyan]

    def __init__(self, parent=None):
        super(PyQtGraphDataPlot, self).__init__(parent)
        self._plot_widget = PlotWidget()
        self._plot_widget.getPlotItem().addLegend()
        self._plot_widget.setBackground((255, 255, 255))
        self._plot_widget.setXRange(0, 10, padding=0)
        vbox = QVBoxLayout()
        vbox.addWidget(self._plot_widget)
        self.setLayout(vbox)

        self._autoscroll = False
        self._color_index = 0
        self._curves = {}
        self._current_vline = None

    def add_curve(self, curve_id, curve_name, data_x, data_y):
        color = QColor(self._colors[self._color_index % len(self._colors)])
        self._color_index += 1
        pen = mkPen(color, width=2)
        # this adds the item to the plot and legend
        plot = self._plot_widget.plot(name=curve_name, pen=pen)
        data_x = numpy.array(data_x)
        data_y = numpy.array(data_y)
        self._curves[curve_id] = {'x': data_x, 'y': data_y, 'plot': plot}

    def remove_curve(self, curve_id):
        curve_id = str(curve_id)
        if curve_id in self._curves:
            self._plot_widget.removeItem(self._curves[curve_id]['plot'])
            del self._curves[curve_id]
            self._update_legend()
           
    def _update_legend(self):
        # clear and rebuild legend (there is no remove item method for the legend...)
        self._plot_widget.clear()
        self._plot_widget.getPlotItem().legend.items = []
        for curve in self._curves.values():
            self._plot_widget.addItem(curve['plot'])
        if self._current_vline:
            self._plot_widget.addItem(self._current_vline)
 
    @Slot(str, list, list)
    def update_values(self, curve_id, x, y):
        curve = self._curves[curve_id]
        curve['x'] = numpy.append(curve['x'], x)
        curve['y'] = numpy.append(curve['y'], y)

    def clear_values(self, curve_id):
        curve = self._curves[curve_id]
        curve['x'] = numpy.array([])
        curve['y'] = numpy.array([])

    def autoscroll(self, enabled=True):
        self._autoscroll = enabled

    def redraw(self):
        for curve in self._curves.values():
            curve['plot'].setData(curve['x'], curve['y'])

    def vline(self, x, color):
        if self._current_vline:
            self._plot_widget.removeItem(self._current_vline)
        self._current_vline = self._plot_widget.addLine(x=x, pen=color)

    def set_xlim(self, limits):
        self._plot_widget.setXRange(limits[0], limits[1], padding=0)

    def set_ylim(self, limits):
        self._plot_widget.setYRange(limits[0], limits[1], padding=0)

    def get_xlim(self):
        rect = self._plot_widget.getPlotItem().getViewBox().targetRect()
        x_range = [rect.left(), rect.right()] 
        return x_range

    def get_ylim(self):
        rect = self._plot_widget.getPlotItem().getViewBox().targetRect()
        y_range = [rect.bottom(), rect.top()] 
        return y_range
