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
import sys

from python_qt_binding.QtCore import QSize, Qt, Signal
from python_qt_binding.QtGui import (QDialog, QGridLayout, QGroupBox, QLabel, 
                                     QHBoxLayout, QLineEdit, QPlainTextEdit, 
                                     QPushButton, QScrollArea,
                                     QSplitter,
                                     QStyle, QToolButton,
                                     QVBoxLayout, QWidget)
import roslaunch
import rospkg
import rospy

from rqt_launch.node_proxy import NodeProxy
from rqt_launch.rxlaunch import (NodeController, NodeGui, NamesSurrogate,
                                 StatusIndicator)

_ID = '/rxlaunch'


class LaunchWindow(QDialog):

    def __init__(self, plugin_context):
        super(LaunchWindow, self).__init__()
        try:
            rp = rospkg.RosPack()
            launchfile = os.path.join(rp.get_path('rqt_launch'), 'test',
                       'test.launch')
        except IndexError:
            sys.stderr.write("Please give a launch file\n")
            sys.exit(1)

        self.run_id = None
        uuid = roslaunch.rlutil.get_or_generate_uuid(self.run_id, True)
        #uuid = 'fake-uuid'
#        print "UUID:", uuid
#        roslaunch.configure_logging(uuid)
#        roslaunch_logger = logging.getLogger("roslaunch")
#        roslaunch_logger.setLevel(logging.DEBUG)

        self.config = roslaunch.config.load_config_default([launchfile], 11311)
        print self.config.summary()
        print "MASTER", self.config.master.uri

        self._load_parameters()

        # Buttons in the header
        self.button_start_all = QPushButton("Start all")
        self.button_start_all.clicked.connect(self.start_all)
        self.button_stop_all = QPushButton("Stop all")
        self.button_stop_all.clicked.connect(self.stop_all)
        header_buttons = QHBoxLayout()
        header_buttons.addWidget(self.button_start_all)
        header_buttons.addWidget(self.button_stop_all)
        header_buttons_box = QGroupBox()
        header_buttons_box.setLayout(header_buttons)

        # Creates the process grid
        self.node_controllers = []
        process_layout = QGridLayout()
        for i, node_config in enumerate(self.config.nodes):
            proxy = NodeProxy(self.run_id, self.config.master.uri, node_config)

            # TODO: consider using QIcon.fromTheme()
            status = StatusIndicator()
            start_button = QPushButton(self.style().standardIcon(QStyle.SP_MediaPlay), "")
            start_button.setIconSize(QSize(16, 16))
            stop_button = QPushButton(self.style().standardIcon(QStyle.SP_MediaStop), "")
            stop_button.setIconSize(QSize(16, 16))
            respawn_toggle = QToolButton()
            respawn_toggle.setIcon(self.style().standardIcon(QStyle.SP_BrowserReload))
            respawn_toggle.setIconSize(QSize(16, 16))
            respawn_toggle.setCheckable(True)
            respawn_toggle.setChecked(proxy.config.respawn)
            spawn_count_label = QLabel("(0)")
            launch_prefix_edit = QLineEdit(proxy.config.launch_prefix)

            gui = NodeGui(status, respawn_toggle, spawn_count_label, launch_prefix_edit)

            node_controller = NodeController(proxy, gui)
            self.node_controllers.append(node_controller)

            #TODO(Isaac) These need to be commented in in order to function as originally intended.
            start_button.clicked.connect(node_controller.start)
            stop_button.clicked.connect(node_controller.stop)

            #resolved_node_name = roslib.names.ns_join(proxy.config.namespace, proxy.config.name)
            rospy.loginfo('loop #%d proxy.config.namespace=%s proxy.config.name=%s', i, proxy.config.namespace, proxy.config.name)
            resolved_node_name = NamesSurrogate.ns_join(proxy.config.namespace, 
                                                        proxy.config.name)

            j = 0
            process_layout.addWidget(status, i, j)
            process_layout.setColumnMinimumWidth(j, 20);  j += 1
            process_layout.addWidget(QLabel(resolved_node_name), i, j);  j += 1
            process_layout.addWidget(spawn_count_label, i, j)
            process_layout.setColumnMinimumWidth(j, 30)              ;  j += 1
            process_layout.setColumnMinimumWidth(j, 30)    ;  j += 1  # Spacer
            process_layout.addWidget(start_button, i, j);  j += 1
            process_layout.addWidget(stop_button, i, j)                           ;  j += 1
            process_layout.addWidget(respawn_toggle, i, j)                        ;  j += 1
            process_layout.setColumnMinimumWidth(j, 20)                           ;  j += 1  # Spacer
            process_layout.addWidget(QLabel(proxy.config.package), i, j)    ;  j += 1
            process_layout.addWidget(QLabel(proxy.config.type), i, j)       ;  j += 1
            process_layout.addWidget(launch_prefix_edit, i, j)                    ;  j += 1

        process_scroll = QScrollArea()
        #process_scroll.setMinimumWidth(process_layout.sizeHint().width())  # Doesn't work properly.  Too small
        process_widget = QWidget()
        process_widget.setLayout(process_layout)
        process_scroll.setWidget(process_widget)

        # Creates the log display area
        self.log_text = QPlainTextEdit()

        # Sets up the overall layout
        process_log_splitter = QSplitter()
        process_log_splitter.setOrientation(Qt.Vertical)
        process_log_splitter.addWidget(process_scroll)
        process_log_splitter.addWidget(self.log_text)
        main_layout = QVBoxLayout()
        main_layout.addWidget(header_buttons_box, stretch=0)
        #main_layout.addWidget(process_scroll, stretch=10)
        #main_layout.addWidget(self.log_text, stretch=30)
        main_layout.addWidget(process_log_splitter)
        self.setLayout(main_layout)

    # Stolen from ROSLaunchRunner
    def _load_parameters(self):
        """
        Load parameters onto the parameter server
        """
        #self.logger.info("load_parameters starting ...")
        config = self.config
        param_server = config.master.get()
        p = None
        try:
            # multi-call style xmlrpc
            param_server_multi = config.master.get_multi()

            # clear specified parameter namespaces
            # #2468 unify clear params to prevent error
            for p in roslaunch.launch._unify_clear_params(config.clear_params):
                if param_server.hasParam(_ID, p)[2]:
                    #printlog("deleting parameter [%s]"%p)
                    param_server_multi.deleteParam(_ID, p)
            r = param_server_multi()
            for code, msg, _ in r:
                if code != 1:
                    raise roslaunch.RLException("Failed to clear parameter: %s"%(msg))

            # multi-call objects are not reusable
            param_server_multi = config.master.get_multi()
            for p in config.params.itervalues():
                # suppressing this as it causes too much spam
                #printlog("setting parameter [%s]"%p.key)
                param_server_multi.setParam(_ID, p.key, p.value)
            r = param_server_multi()
            for code, msg, _ in r:
                if code != 1:
                    raise roslaunch.RLException("Failed to set parameter: %s"%(msg))
        except roslaunch.RLException:
            raise
        except Exception, e:
            #printerrlog("load_parameters: unable to set parameters (last param was [%s]): %s"%(p,e))
            print("load_parameters: unable to set parameters (last param was [%s]): %s"%(p,e))
            raise #re-raise as this is fatal
        #self.logger.info("... load_parameters complete")
        print("... load_parameters complete")

    def start_all(self):
        print "Starting all nodes"
        for n in self.node_controllers:
            n.start(restart=False)

    def stop_all(self):
        print "Stopping all nodes"
        for n in self.node_controllers:
            n.stop()

    def check_process_statuses(self):
        for n in self.node_controllers:
            n.check_process_status()

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
