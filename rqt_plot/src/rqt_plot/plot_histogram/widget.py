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

try:
    from cStringIO import StringIO
except ImportError:
    import StringIO
import collections
import os

import cv2
import numpy as np

from cv_bridge import CvBridge
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Slot
from python_qt_binding.QtGui import QIcon, QWidget
import rospkg
import rospy
from rqt_py_common.topic_completer import TopicCompleter
from sensor_msgs.msg import Image

from .histogram_plot import HistogramPlot
from ..rosplot import ROSArrayData


class PlotHistogramWidget(QWidget):

    _redraw_interval = 40

    def __init__(self, topic_name=None):
        super(PlotHistogramWidget, self).__init__()
        self.setObjectName('PlotHistogramWidget')
        rp = rospkg.RosPack()
        ui_file = os.path.join(
            rp.get_path('rqt_plot'), 'resource', 'plot_histogram.ui')
        loadUi(ui_file, self)
        self.cv_bridge = CvBridge()
        self.subscribe_topic_button.setIcon(QIcon.fromTheme('add'))
        self.pause_button.setIcon(QIcon.fromTheme('media-playback-pause'))
        self.clear_button.setIcon(QIcon.fromTheme('edit-clear'))
        self.data_plot = HistogramPlot(self)
        self.data_plot_layout.addWidget(self.data_plot)
        self._topic_completer = TopicCompleter(self.topic_edit)
        self._topic_completer.update_topics()
        self.topic_edit.setCompleter(self._topic_completer)
        self.data_plot.dropEvent = self.dropEvent
        self.data_plot.dragEnterEvent = self.dragEnterEvent
        self._start_time = rospy.get_time()
        self._rosdata = None
        if topic_name:
            self.subscribe_topic(topic_name)
        self._update_plot_timer = QTimer(self)
        self._update_plot_timer.timeout.connect(self.update_plot)
        self._update_plot_timer.start(self._redraw_interval)

    @Slot('QDropEvent*')
    def dropEvent(self, event):
        if event.mimeData().hasText():
            topic_name = str(event.mimeData().text())
        else:
            droped_item = event.source().selectedItems()[0]
            topic_name = str(droped_item.data(0, Qt.UserRole))
        self.subscribe_topic(topic_name)

    @Slot()
    def on_topic_edit_returnPressed(self):
        if self.subscribe_topic_button.isEnabled():
            self.subscribe_topic(str(self.topic_edit.text()))

    @Slot()
    def on_subscribe_topic_button_clicked(self):
        self.subscribe_topic(str(self.topic_edit.text()))

    def subscribe_topic(self, topic_name):
        self.topic_with_field_name = topic_name
        self.pub_fig = rospy.Publisher(
            os.path.join(topic_name, 'figure_image'), Image, queue_size=1)
        if not self._rosdata:
            self._rosdata = ROSArrayData(topic_name, self._start_time)
        else:
            if self._rosdata != topic_name:
                self._rosdata.close()
                self.data_plot.clear()
                self._rosdata = ROSArrayData(topic_name, self._start_time)
            else:
                rospy.logwarn("%s is already subscribed", topic_name)

    def enable_timer(self, enabled=True):
        if enabled:
            self._update_plot_timer.start(self._redraw_interval)
        else:
            self._update_plot_timer.stop()

    @Slot()
    def on_clear_button_clicked(self):
        self.data_plot.clear()

    @Slot(bool)
    def on_pause_button_clicked(self, checked):
        self.enable_timer(not checked)

    def update_plot(self):
        if not self._rosdata:
            return
        data_x, data_y = self._rosdata.next()

        if len(data_y) == 0:
            return
        axes = self.data_plot._canvas.axes
        axes.cla()
        if isinstance(data_y[-1], collections.Sequence):
            xs = data_y[-1]
            pos = np.arange(len(xs))
            widths = [1] * len(xs)
            axes.set_xlim(xmin=0, xmax=len(xs))
        else:
            rospy.logerr(
                "Topic/Field name '%s' has unsupported '%s' type."
                "Tuple/List of float values are only supported."
                % (self.topic_with_field_name, self._rosdata.sub.data_class))
            return
        for p, x, w in zip(pos, xs, widths):
            axes.bar(p, x, color='r', align='center', width=w)
        axes.legend([self.topic_with_field_name], prop={'size': '8'})
        self.data_plot._canvas.draw()

        if self.pub_fig.get_num_connections() > 0:
            buf = StringIO()
            self.data_plot._canvas.figure.savefig(buf, format="png")
            buf.seek(0)
            img_array = np.asarray(bytearray(buf.read()), dtype=np.uint8)
            img = cv2.imdecode(img_array, cv2.CV_LOAD_IMAGE_COLOR)
            self.pub_fig.publish(self.cv_bridge.cv2_to_imgmsg(img, "bgr8"))
