# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
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
#
# Author: Isaac Saito

import os

from python_qt_binding import loadUi
from python_qt_binding.QtCore import QSize, Qt, Signal
from python_qt_binding.QtGui import (QDialog, QGridLayout, QGroupBox, QLabel,
                                     QHBoxLayout, QLineEdit, QPlainTextEdit,
                                     QPushButton, QScrollArea, QSplitter,
                                     QStyle, QToolButton, QVBoxLayout, QWidget)
import rospkg
import rospy

from rqt_launch.node_proxy import NodeProxy
from rqt_launch.node_controller import NodeController
from rqt_launch.node_gui import NodeGui
from rqt_launch.name_surrogate import NamesSurrogate
from rqt_launch.status_indicator import StatusIndicator


class LaunchWidget(QDialog):
    """
    #TODO: comment
    """

    def __init__(self, config, parent):
        """
        @type parent: LaunchMain
        @type config: ?
        """
        super(LaunchWidget, self).__init__()
        self._parent = parent
        self._config = config

        self.run_id = None
        rospy.loginfo(self._config.summary())
        #rospy.loginfo("MASTER", self._config.master.uri)  # Sheds error.
        print "MASTER", self._config.master.uri

        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_launch'),
                               'resource', 'launch_widget.ui')
        loadUi(ui_file, self)

        self._pushbutton_start_stop_all.clicked.connect(self._parent.start_all)

        # Creates the process grid
        self._node_controllers = []
        process_layout = QGridLayout()
        # Loop per node
        for i, node_config in enumerate(self._config.nodes):
            _proxy = NodeProxy(self.run_id, self._config.master.uri,
                              node_config)

            # TODO: consider using QIcon.fromTheme()
            status = StatusIndicator()
            start_button = QPushButton(self.style().standardIcon(
                                                      QStyle.SP_MediaPlay), "")
            start_button.setIconSize(QSize(16, 16))
            stop_button = QPushButton(self.style().standardIcon(
                                                      QStyle.SP_MediaStop), "")
            stop_button.setIconSize(QSize(16, 16))
            respawn_toggle = QToolButton()
            respawn_toggle.setIcon(self.style().standardIcon(
                                                      QStyle.SP_BrowserReload))
            respawn_toggle.setIconSize(QSize(16, 16))
            respawn_toggle.setCheckable(True)
            respawn_toggle.setChecked(_proxy.config.respawn)
            spawn_count_label = QLabel("(0)")
            launch_prefix_edit = QLineEdit(_proxy.config.launch_prefix)

            gui = NodeGui(status, respawn_toggle, spawn_count_label,
                          launch_prefix_edit)

            node_controller = NodeController(_proxy, gui)
            self._node_controllers.append(node_controller)

            #TODO(Isaac) These need to be commented in in order to function as
            # originally intended.
            start_button.clicked.connect(node_controller.start)
            stop_button.clicked.connect(node_controller.stop)

            #resolved_node_name = roslib.names.ns_join(_proxy.config.namespace,
            # _proxy.config.name)
            rospy.loginfo('loop #%d _proxy.config.namespace=%s ' +
                          '_proxy.config.name=%s',
                          i, _proxy.config.namespace, _proxy.config.name)
            resolved_node_name = NamesSurrogate.ns_join(_proxy.config.namespace,
                                                        _proxy.config.name)

            j = 0
            process_layout.addWidget(status, i, j)
            process_layout.setColumnMinimumWidth(j, 20);  j += 1
            process_layout.addWidget(QLabel(resolved_node_name), i, j);  j += 1
            process_layout.addWidget(spawn_count_label, i, j)
            process_layout.setColumnMinimumWidth(j, 30)              ;  j += 1
            process_layout.setColumnMinimumWidth(j, 30)    ;  j += 1  # Spacer
            process_layout.addWidget(start_button, i, j);  j += 1
            process_layout.addWidget(stop_button, i, j) ;  j += 1
            process_layout.addWidget(respawn_toggle, i, j) ;  j += 1
            process_layout.setColumnMinimumWidth(j, 20) ;  j += 1  # Spacer
            process_layout.addWidget(QLabel(_proxy.config.package), i, j);j += 1
            process_layout.addWidget(QLabel(_proxy.config.type), i, j);  j += 1
            process_layout.addWidget(launch_prefix_edit, i, j)  ;  j += 1

        self._parent.set_node_controllers(self._node_controllers)
        #process_scroll.setMinimumWidth(process_layout.sizeHint().width())
        # Doesn't work properly.  Too small
        self._process_widget.setLayout(process_layout)

        # Creates the log display area
        self.log_text = QPlainTextEdit()

        # Sets up the overall layout
        process_log_splitter = QSplitter()
        process_log_splitter.setOrientation(Qt.Vertical)
        process_log_splitter.addWidget(self.log_text)
        main_layout = QVBoxLayout()
        #main_layout.addWidget(process_scroll, stretch=10)
        #main_layout.addWidget(self.log_text, stretch=30)
        main_layout.addWidget(process_log_splitter)
        self.setLayout(main_layout)

    def shutdown(self):
        #TODO: Needs implemented. Trigger dynamic_reconfigure to unlatch
        #            subscriber.
        pass

    def save_settings(self, plugin_settings, instance_settings):
        #instance_settings.set_value('splitter', self._splitter.saveState())
        pass

    def restore_settings(self, plugin_settings, instance_settings):
#        if instance_settings.contains('splitter'):
#            self._splitter.restoreState(instance_settings.value('splitter'))
#        else:
#            self._splitter.setSizes([100, 100, 200])
        pass
