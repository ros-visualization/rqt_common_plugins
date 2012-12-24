#!/usr/bin/env python

# Copyright (c) 2011, Ye Cheng, Dorian Scholz
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

from python_qt_binding import QT_BINDING, QT_BINDING_VERSION
if QT_BINDING == 'pyside':
    try:
        from pkg_resources import parse_version
    except:
        import re

        def parse_version(s):
            return [int(x) for x in re.sub(r'(\.0+)*$', '', s).split('.')]
    if parse_version(QT_BINDING_VERSION) <= parse_version('1.1.2'):
        raise ImportError('A PySide version newer than 1.1.0 is required.')

from python_qt_binding.QtCore import Slot, Qt
from python_qt_binding.QtGui import QWidget, QVBoxLayout, QSizePolicy, QColor

import operator
import matplotlib
if matplotlib.__version__ < '1.1.0':
    raise ImportError('A newer matplotlib is required (at least 1.1.0)')

try:
    from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
except ImportError:
    # work around bug in dateutil
    import sys
    import thread
    sys.modules['_thread'] = thread
    from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt4agg import NavigationToolbar2QTAgg as NavigationToolbar
from matplotlib.figure import Figure

import numpy


class MatDataPlot(QWidget):
    class Canvas(FigureCanvas):
        """Ultimately, this is a QWidget (as well as a FigureCanvasAgg, etc.)."""
        def __init__(self, parent=None):
            super(MatDataPlot.Canvas, self).__init__(Figure())
            self.axes = self.figure.add_subplot(111)
            self.axes.grid(True, color='gray')
            self.figure.tight_layout()
            self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            self.updateGeometry()

        def resizeEvent(self, event):
            super(MatDataPlot.Canvas, self).resizeEvent(event)
            self.figure.tight_layout()

    _colors = [Qt.red, Qt.blue, Qt.magenta, Qt.cyan, Qt.green, Qt.darkYellow, Qt.black, Qt.darkRed, Qt.gray, Qt.darkCyan]

    def __init__(self, parent=None):
        super(MatDataPlot, self).__init__(parent)
        self._canvas = MatDataPlot.Canvas()
        self._toolbar = NavigationToolbar(self._canvas, self._canvas)
        vbox = QVBoxLayout()
        vbox.addWidget(self._toolbar)
        vbox.addWidget(self._canvas)
        self.setLayout(vbox)

        self._color_index = 0
        self._curves = {}

    def add_curve(self, curve_id, curve_name, x, y):
        color = QColor(self._colors[self._color_index % len(self._colors)])
        self._color_index += 1
        line = self._canvas.axes.plot([], [], label=curve_name, linewidth=1, picker=5, color=color.name())[0]
        self._curves[curve_id] = ([], [], line, [None, None])
        self.update_values(curve_id, x, y)
        self._update_legend()

    def remove_curve(self, curve_id):
        curve_id = str(curve_id)
        if curve_id in self._curves:
            self._curves[curve_id][2].remove()
            del self._curves[curve_id]
            self._update_legend()

    def _update_legend(self):
        handles, labels = self._canvas.axes.get_legend_handles_labels()
        if handles:
            hl = sorted(zip(handles, labels), key=operator.itemgetter(1))
            handles, labels = zip(*hl)
        self._canvas.axes.legend(handles, labels, loc='upper left')

    @Slot(str, list, list)
    def update_values(self, curve_id, x, y):
        data_x, data_y, line, range_y = self._curves[curve_id]
        data_x.extend(x)
        data_y.extend(y)
        line.set_data(data_x, data_y)
        if y:
            ymin = min(y)
            if range_y[0]:
                ymin = min(ymin, range_y[0])
            range_y[0] = ymin
            ymax = max(y)
            if range_y[1]:
                ymax = max(ymax, range_y[1])
            range_y[1] = ymax

    def redraw(self):
        self._canvas.axes.grid(True, color='gray')
        # Set axis bounds
        ymin = ymax = None
        xmax = 0
        for curve in self._curves.values():
            data_x, _, _, range_y = curve
            if len(data_x) == 0:
                continue

            xmax = max(xmax, data_x[-1])

            if ymin is None:
                ymin = range_y[0]
                ymax = range_y[1]
            else:
                ymin = min(range_y[0], ymin)
                ymax = max(range_y[1], ymax)

            # pad the min/max
            delta = max(ymax - ymin, 0.1)
            ymin -= .05 * delta
            ymax += .05 * delta

        if ymin is not None:
            self._canvas.axes.set_xbound(lower=xmax - 5, upper=xmax)
            self._canvas.axes.set_ybound(lower=ymin, upper=ymax)

        self._canvas.draw()
