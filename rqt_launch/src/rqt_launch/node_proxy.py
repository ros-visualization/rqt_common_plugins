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

from python_qt_binding.QtCore import Signal
from python_qt_binding.QtGui import (QDialog, QGridLayout, QGroupBox, QLabel,
                                     QHBoxLayout, QPushButton, QSplitter,
                                     QVBoxLayout, QWidget)
from roslaunch import nodeprocess


class NodeProxy(object):
    """
    Works as a proxy between ROS Node & GUI.
    """

    __slots__ = ['run_id', 'master_uri', 'config', 'process']

    def __init__(self, run_id, master_uri, config):
        self.run_id = run_id
        self.master_uri = master_uri
        self.config = config

        self.recreate_process()

    # LocalProcess.is_alive() does not do what you would expect
    def is_running(self):
        return self.process.started and self.process.is_alive()

    def has_died(self):
        return (self.process.started and
                not self.process.stopped and
                not self.process.is_alive())

    def recreate_process(self):
        """
        Create and set roslaunch.nodeprocess.LocalProcess to member variable.
        """
        self.process = nodeprocess.create_node_process(
                                     self.run_id, self.config, self.master_uri)
