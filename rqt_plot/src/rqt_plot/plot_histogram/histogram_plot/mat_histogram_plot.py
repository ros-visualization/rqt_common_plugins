# -----------------------------------------------------------------------------
# Software License Agreement (BSD License)
#
#  Copyright (c) 2014-2017, JSK Laboratory.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/o2r other materials provided
#     with the distribution.
#   * Neither the name of the JSK Laboratory nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
# -----------------------------------------------------------------------------

from python_qt_binding import QT_BINDING, QT_BINDING_VERSION

try:
    from pkg_resources import parse_version
except ImportError:
    import re

    def parse_version(s):
        return [int(x) for x in re.sub(r'(\.0+)*$', '', s).split('.')]

if QT_BINDING == 'pyside':
    qt_binding_version = QT_BINDING_VERSION.replace('~', '-')
    if parse_version(qt_binding_version) <= parse_version('1.1.2'):
        raise ImportError('A PySide version newer than 1.1.0 is required.')

from python_qt_binding.QtCore import qVersion
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QSizePolicy

import matplotlib
if qVersion().startswith('5.'):
    if QT_BINDING == 'pyside':
        if parse_version(matplotlib.__version__) < parse_version('2.1.0'):
            raise ImportError('A newer matplotlib is required (at least 2.1.0 for PySide 2)')
    if parse_version(matplotlib.__version__) < parse_version('1.4.0'):
        raise ImportError('A newer matplotlib is required (at least 1.4.0 for Qt 5)')
    try:
        matplotlib.use('Qt5Agg')
        from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
    except ImportError:
        # work around bug in dateutil
        import sys
        import thread
        sys.modules['_thread'] = thread
        from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
    from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
elif qVersion().startswith('4.'):
    if parse_version(matplotlib.__version__) < parse_version('1.1.0'):
        raise ImportError('A newer matplotlib is required (at least 1.1.0 for Qt 4)')
    try:
        matplotlib.use('Qt4Agg')
        from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
    except ImportError:
        # work around bug in dateutil
        import sys
        import thread
        sys.modules['_thread'] = thread
        from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
    try:
        from matplotlib.backends.backend_qt4agg import NavigationToolbar2QTAgg as NavigationToolbar
    except ImportError:
        from matplotlib.backends.backend_qt4agg import NavigationToolbar2QT as NavigationToolbar
else:
    raise NotImplementedError('Unsupport Qt version: %s' % qVersion())

from matplotlib.figure import Figure


class MatHistogramPlot(QWidget):

    class Canvas(FigureCanvas):

        def __init__(self, parent=None):
            super(MatHistogramPlot.Canvas, self).__init__(Figure())
            self.axes = self.figure.add_subplot(111)
            self.figure.tight_layout()
            self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            self.updateGeometry()

        def resizeEvent(self, event):
            super(MatHistogramPlot.Canvas, self).resizeEvent(event)
            self.figure.tight_layout()

    def __init__(self, parent=None):
        super(MatHistogramPlot, self).__init__(parent)
        self._canvas = MatHistogramPlot.Canvas()
        self._toolbar = NavigationToolbar(self._canvas, self._canvas)
        vbox = QVBoxLayout()
        vbox.addWidget(self._toolbar)
        vbox.addWidget(self._canvas)
        self.setLayout(vbox)

    def redraw(self):
        pass

    def clear(self):
        self._canvas.axes.cla()
        self._canvas.draw()
