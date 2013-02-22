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

import roslaunch
import rospkg

from rqt_launch.launch_widget import LaunchWidget
from rqt_py_common.rqt_roscomm_util import RqtRoscommUtil


class LaunchMain(object):
    def __init__(self, plugin_context):
        super(LaunchMain, self).__init__()
        self._plugin_context = plugin_context

        #TODO: launch file name is obtained here. This should be replaced by
        # generic method that enables choosing .launch files that exist in
        # the local file system.
        rp = rospkg.RosPack()
        try:
            launchfile = os.path.join(rp.get_path(
                          #'rqt_launch'), 'test', 'test.launch')
                          #'runtime_monitor'), 'test', 'runtime_mon_test.launch')
                          #'gazebo_worlds'), 'launch', 'empty_world_paused.launch')
                          'turtle_tf'), 'launch', 'turtle_tf_demo.launch')
        except IndexError:
            sys.stderr.write("Please give a launch file\n")
            sys.exit(1)

        self._config = roslaunch.config.load_config_default([launchfile],
                                                            11311)
        self._mainwidget = LaunchWidget(self._config, self)

        self.run_id = None

        print self._config.summary()
        print "MASTER", self._config.master.uri

        RqtRoscommUtil.load_parameters(self._config, '/rqt_launch')

    def get_widget(self):
        return self._mainwidget

    def set_node_controllers(self, node_controllers):
        self._node_controllers = node_controllers

    def start_all(self):
        print "Starting all nodes"
        for n in self._node_controllers:
            n.start(restart=False)

    def stop_all(self):
        print "Stopping all nodes"
        for n in self._node_controllers:
            n.stop()

    def check_process_statuses(self):
        for n in self._node_controllers:
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
