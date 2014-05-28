# Software License Agreement (BSD License)
#
# Copyright (c) 2014, Austin Hendrix, Stanford University
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Design notes:
#
# The original rxbag plot widget displays the plot
#
# It has a settings menu which allows the user to add subplots
#  and assign arbitrary fields to subplots. possibly many fields in a single
#  subplot, or one field per subplot, or any othe rcombination
#  in particular, it is possible to add the same field to multiple subplots
# It doesn't appear able to add fields from different topics to the same plot
#  Since rqt_plot can do this, and it's useful for our application, it's worth
#  thinking about. If it makes the UI too cluttered, it may not be worth it
#  If it isn't possible due to the architecture of rqt_bag, it may not be
#  worth it
#
# Thoughts on new design:
#  a plottable field is anything which is either numeric, or contains
#  at least one plottable subfield (this is useful for enabling/disabling all
#  of the fields of a complex type)
#
#  for simple messages, we could display a tree view of the message fields
#  on top of the plot, along with the color for each plottable field. This gets
#  unweildy for large messages, because they'll use up too much screen space
#  displaying the topic list
#
#  It may be better to display the topic list for a plot as a dockable widget,
#  and be able to dismiss it when it isn't actively in use, similar to the
#  existing rxbag plot config window
#
#  The plot should be dockable and viewable. If it's a separate window, it
#  doesn't make sense to make it align with the timeline. This could be done
#  if someone wanted to implement a separate timeline view

from rqt_bag import MessageView

from python_qt_binding.QtGui import QWidget, QSplitter, QVBoxLayout, QPushButton

# imports for adwilson's implementation
import os
import codecs
import rospkg
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, qWarning
from python_qt_binding.QtGui import QTreeWidget, QTreeWidgetItem, QSizePolicy, QDoubleValidator
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt4agg import NavigationToolbar2QT as NavigationToolbar
import matplotlib.pyplot as plt


from rqt_py_common.data_plot import DataPlot

import rospy

class PlotView(MessageView):
    """
    Popup plot viewer
    """
    name = 'Plot'

    def __init__(self, timeline, parent, topic):
        super(PlotView, self).__init__(timeline, topic)

        self.plot_widget = PlotWidget(timeline, parent, topic)

        parent.layout().addWidget(self.plot_widget)
        self._fields = set()

        #TODO: set start and end timestamps from area of interest
        #      we should probably implement this by having the timeline widget
        #      emit a signal whenever the region of interest changes, and then
        #      have the plugin creation code connect that signal to our
        #      plugin
        #
        #      or we model the interaction explicitly, where the timeline has
        #      a list of listeners, and calls a callback on each of them
        #      explicitly
        #
        # in general, this suggests that more of the interaction

    def message_viewed(self, bag, msg_details):
        """
        refreshes the plot
        """
        _, msg, t = msg_details[:3]

        if not msg:
            #self.set_plot(None, topic, 'no message')
            pass
        else:
            #self.set_plot(msg, topic, msg.header.stamp)
            pass

        # TODO: initialize this on startup
        if self.plot_widget.bag == None:
            self.plot_widget.bag = bag

        if t is None:
            self.message_cleared()
        else:
            self.plot_widget.message_tree.set_message(msg)
            self.plot_widget.set_cursor((t-self.plot_widget.start_stamp).to_sec())
            self.plot_widget.plot.redraw()

    def message_cleared(self):
        pass

class PlotWidget(QWidget):
    def __init__(self, timeline, parent, topic):
        super(PlotWidget, self).__init__(parent)
        self.setObjectName('PlotWidget')

        self.timeline = timeline
        msg_type = self.timeline.get_datatype(topic)
        self.msgtopic = topic
        self.start_stamp = self.timeline._get_start_stamp()
        self.end_stamp = self.timeline._get_end_stamp()
        self.limits = [0,(self.end_stamp-self.start_stamp).to_sec()]

        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_bag_plugins'), 'resource', 'plot.ui')
        loadUi(ui_file, self)
        self.message_tree = MessageTree(msg_type, self)
        self.data_tree_layout.addWidget(self.message_tree)
        self.auto_res.stateChanged.connect(self.autoChanged) # TODO: make this a dropdown with choices for "Auto", "Full" and "Custom"

        # TODO: pull or listen to start and end time from timeline region
        self.start_time.editingFinished.connect(self.settingsChanged)
        self.end_time.editingFinished.connect(self.settingsChanged)
        self.resolution.editingFinished.connect(self.settingsChanged)
        self.start_time.setValidator(QDoubleValidator(0.0,1000.0,2,self.start_time))
        self.end_time.setValidator(QDoubleValidator(0.0,1000.0,2,self.end_time))
        self.resolution.setValidator(QDoubleValidator(0.0,1000.0,6,self.resolution))

        # set start and end time from limits
        self.start_time.setText('0.0')
        self.end_time.setText(str(round((self.end_stamp-self.start_stamp).to_sec(),2)))
        self.resolution.setText(str(round((self.end_stamp-self.start_stamp).to_sec()/200.0,5)))

        self.plot = DataPlot(self)
        self.data_plot_layout.addWidget(self.plot)

        self._config_button = QPushButton("Configure Plot")
        self.plot_toolbar_layout.addWidget(self._config_button)
        self._config_button.clicked.connect(self.plot.doSettingsDialog)

        self.set_cursor(0)
        self.plot.redraw()

        self.paths_on = []
        self._lines = None
        self.bag = None

    def set_cursor(self, position):
        self.plot.vline(0, color='r')

    def add_plot(self, path):
        limits = self.plot.get_xlim()
        limits = self.limits # TODO: remove when get_xlim works
        _limits = limits
        if self.auto_res.isChecked():
            timestep = round((limits[1]-limits[0])/200.0,5)
        else:
            timestep = float(self.resolution.text()) 
        self.resolution.setText(str(timestep))

        if limits[0]<0:
            limits = [0.0,limits[1]]
        if limits[1]>(self.end_stamp-self.start_stamp).to_sec():
            limits = [limits[0],(self.end_stamp-self.start_stamp).to_sec()]

        self.load_data(rospy.Duration.from_sec(limits[0]),rospy.Duration.from_sec((self.end_stamp-self.start_stamp).to_sec()-limits[1]))
        y = []
        x = []
        for entry in self.msgdata:
            if x==[] or (entry[2]-self.start_stamp).to_sec()-x[-1] >= timestep:
                y.append(eval('entry[1].' + path))
                x.append((entry[2]-self.start_stamp).to_sec())

        self.plot.add_curve(path, path, x, y)

        # autoscale Y
        #self.plot.autoscale_x
        # set X scale
        #self.plot.set_xlim(_limits)
        self.plot.redraw()
        #self.ax.plot(x,y, 'o-',markersize=3,label=path)
        #self.paths_on.append(path)

        #self._lines = self.ax.lines[0]
        #del self.ax.lines[0]
        #self.ax.legend()
        #self.ax.relim()
        #self.ax.set_autoscale_on(True)
        #self.ax.autoscale_view(False,False,True)
        #self.ax.set_xlim(_limits)
        #self.ax.lines.insert(0,self._lines)
        #self.canvas.draw()

    def update_plot(self, limits, timestep):
        self.resolution.setText(str(timestep))
        self.start_time.setText(str(round(limits[0],2)))
        self.end_time.setText(str(round(limits[1],2)))
        self.ax.set_xlim(limits)

        if limits[0]<0:
            limits = [0.0,limits[1]]
        if limits[1]>(self.end_stamp-self.start_stamp).to_sec():
            limits = [limits[0],(self.end_stamp-self.start_stamp).to_sec()]


        if hasattr(self, 'paths_on') and len(self.paths_on)>0:
            self.load_data(rospy.Duration.from_sec(limits[0]),rospy.Duration.from_sec((self.end_stamp-self.start_stamp).to_sec()-limits[1]))
         
            y = [[] for i in range(len(self.paths_on))]
            x = [[] for i in range(len(self.paths_on))]

            for entry in self.msgdata:
                path_index = 0
                for path in self.paths_on:
                    if x[path_index]==[] or (entry[2]-self.start_stamp).to_sec()-x[path_index][-1] >= timestep:
                        y[path_index].append(eval('entry[1].' + path))
                        x[path_index].append((entry[2]-self.start_stamp).to_sec())
                    path_index +=  1
            path_index = 0
            for path in self.paths_on:
                self.ax.lines[path_index+1].set_data(x[path_index],y[path_index])
                path_index += 1
            self.canvas.draw()

    def remove_plot(self, path):
        self.plot.remove_curve(path)
        self.plot.redraw()
        #del self.ax.lines[self.paths_on.index(path)+1]
        #del self.paths_on[self.paths_on.index(path)]

        #self._lines = self.ax.lines[0]
        #del self.ax.lines[0]
        #self.ax.relim()
        #self.ax.set_autoscale_on(True)
        #self.ax.autoscale_view(False,False,True)
        #if self.paths_on == []:
        #    self.ax.legend_ = None
        #else:        
        #    self.ax.legend()
        #self.ax.lines.insert(0,self._lines)
        #self.canvas.draw()

    def load_data(self,startoffset,endoffset):
        self.msgdata=self.bag.read_messages(self.msgtopic,self.start_stamp+startoffset,self.end_stamp-endoffset)

    def on_motion(self, event):
        qWarning("PlotWidget.on_motion")
        limits = self.ax.get_xlim()
        if self.auto_res.isChecked():
            timestep = round((limits[1]-limits[0])/200.0,5)
        else:
            timestep = float(self.resolution.text())
        self.update_plot(limits, timestep)

    def settingsChanged(self):
        limits = [float(self.start_time.text()),float(self.end_time.text())]
        if self.auto_res.isChecked():
            timestep = round((limits[1]-limits[0])/200.0,5)
        else:
            timestep = float(self.resolution.text())
        self.update_plot(limits, timestep)

    def autoChanged(self, state):
        if state==2:
            self.resolution.setDisabled(True) 
            limits = self.ax.get_xlim()
            timestep = round((limits[1]-limits[0])/200.0,5)
            self.update_plot(limits, timestep)   
        else:
            self.resolution.setDisabled(False)

    def home(self):
        limits = [0,(self.end_stamp-self.start_stamp).to_sec()]
        self.ax.relim()
        self.ax.set_autoscale_on(True)
        self.ax.autoscale_view(False,False,True)
        if self.auto_res.isChecked():
            timestep = round((limits[1]-limits[0])/200.0,5)
        else:
            timestep = float(self.resolution.text())
        self.update_plot(limits, timestep)



class MessageTree(QTreeWidget):
    def __init__(self, msg_type, parent):
        super(MessageTree, self).__init__(parent)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setHeaderHidden(True)
        self.itemChanged.connect(self.handleChanged)
        self._msg_type = msg_type
        self._msg = None

        self._expanded_paths = None
        self._checked_states = set()
        self.plot_list = set()

        # populate the tree from the message type


    @property
    def msg(self):
        return self._msg

    def set_message(self, msg):
        # Remember whether items were expanded or not before deleting
        if self._msg:
            for item in self.get_all_items():
                path = self.get_item_path(item)
                if item.isExpanded():
                    self._expanded_paths.add(path)
                elif path in self._expanded_paths:
                    self._expanded_paths.remove(path)
                if item.checkState(0)==Qt.Checked:
                    self._checked_states.add(path)
                elif path in self._checked_states:
                    self._checked_states.remove(path)
            self.clear()
        if msg:
            # Populate the tree
            self._add_msg_object(None, '', '', msg, msg._type)

            if self._expanded_paths is None:
                self._expanded_paths = set()
            else:
                # Expand those that were previously expanded, and collapse any paths that we've seen for the first time
                for item in self.get_all_items():
                    path = self.get_item_path(item)
                    if path in self._expanded_paths:
                        item.setExpanded(True)
                    else:
                        item.setExpanded(False)
        self._msg = msg
        self.update()

    def get_item_path(self, item):
        return item.data(0, Qt.UserRole)[0].replace(' ', '')  # remove spaces that may get introduced in indexing, e.g. [  3] is [3]

    def get_all_items(self):
        items = []
        try:
            root = self.invisibleRootItem()
            self.traverse(root, items.append)
        except Exception:
            # TODO: very large messages can cause a stack overflow due to recursion
            pass
        return items

    def traverse(self, root, function):
        for i in range(root.childCount()):
            child = root.child(i)
            function(child)
            self.traverse(child, function)

    def _add_msg_object(self, parent, path, name, obj, obj_type):
        label = name

        if hasattr(obj, '__slots__'):
            subobjs = [(slot, getattr(obj, slot)) for slot in obj.__slots__]
        elif type(obj) in [list, tuple]:
            len_obj = len(obj)
            if len_obj == 0:
                subobjs = []
            else:
                w = int(math.ceil(math.log10(len_obj)))
                subobjs = [('[%*d]' % (w, i), subobj) for (i, subobj) in enumerate(obj)]
        else:
            subobjs = []

        plotitem=False
        if type(obj) in [int, long, float]:
            plotitem=True
            if type(obj) == float:
                obj_repr = '%.6f' % obj
            else:
                obj_repr = str(obj)

            if obj_repr[0] == '-':
                label += ': %s' % obj_repr
            else:
                label += ':  %s' % obj_repr

        elif type(obj) in [str, bool, int, long, float, complex, rospy.Time]:
            # Ignore any binary data
            obj_repr = codecs.utf_8_decode(str(obj), 'ignore')[0]

            # Truncate long representations
            if len(obj_repr) >= 50:
                obj_repr = obj_repr[:50] + '...'

            label += ': ' + obj_repr
        item = QTreeWidgetItem([label])
        if name == '':
            pass
        elif path.find('.') == -1 and path.find('[') == -1:
            self.addTopLevelItem(item)
        else:
            parent.addChild(item)
        if plotitem == True:
            if path.replace(' ', '') in self._checked_states:
                item.setCheckState (0, Qt.Checked)
            else:
                item.setCheckState (0, Qt.Unchecked)
        item.setData(0, Qt.UserRole, (path, obj_type))


        for subobj_name, subobj in subobjs:
            if subobj is None:
                continue

            if path == '':
                subpath = subobj_name  # root field
            elif subobj_name.startswith('['):
                subpath = '%s%s' % (path, subobj_name)  # list, dict, or tuple
            else:
                subpath = '%s.%s' % (path, subobj_name)  # attribute (prefix with '.')

            if hasattr(subobj, '_type'):
                subobj_type = subobj._type
            else:
                subobj_type = type(subobj).__name__

            self._add_msg_object(item, subpath, subobj_name, subobj, subobj_type)

    def handleChanged(self, item, column):
        if item.data(0, Qt.UserRole)==None:
            pass
        else:
            path = self.get_item_path(item)
            if item.checkState(column) == Qt.Checked:
                if path not in self.plot_list:
                    self.plot_list.add(path)
                    self.parent().parent().parent().add_plot(path)
            if item.checkState(column) == Qt.Unchecked:
                if path in self.plot_list:
                    self.plot_list.remove(path)
                    self.parent().parent().parent().remove_plot(path)
