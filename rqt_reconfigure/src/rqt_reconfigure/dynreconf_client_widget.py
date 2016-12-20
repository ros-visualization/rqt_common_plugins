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
# Author: Isaac Saito, Ze'ev Klapow

import rospy

from python_qt_binding.QtCore import QMargins
from python_qt_binding.QtGui import QIcon
from python_qt_binding.QtWidgets import (QFileDialog, QHBoxLayout,
                                         QPushButton, QWidget)
from .param_editors import EditorWidget
from .param_groups import GroupWidget, find_cfg
from .param_updater import ParamUpdater

import yaml


class DynreconfClientWidget(GroupWidget):
    """
    A wrapper of dynamic_reconfigure.client instance.
    Represents a widget where users can view and modify ROS params.
    """

    def __init__(self, reconf, node_name):
        """
        :type reconf: dynamic_reconfigure.client
        :type node_name: str
        """

        group_desc = reconf.get_group_descriptions()
        rospy.logdebug('DynreconfClientWidget.group_desc=%s', group_desc)
        super(DynreconfClientWidget, self).__init__(ParamUpdater(reconf),
                                                    group_desc, node_name)

        # Save and load buttons
        self.button_widget = QWidget(self)
        self.button_header = QHBoxLayout(self.button_widget)
        self.button_header.setContentsMargins(QMargins(0, 0, 0, 0))

        self.load_button = QPushButton()
        self.save_button = QPushButton()

        self.load_button.setIcon(QIcon.fromTheme('document-open'))
        self.save_button.setIcon(QIcon.fromTheme('document-save'))

        self.load_button.clicked[bool].connect(self._handle_load_clicked)
        self.save_button.clicked[bool].connect(self._handle_save_clicked)

        self.button_header.addWidget(self.save_button)
        self.button_header.addWidget(self.load_button)

        self.setMinimumWidth(150)

        self.reconf = reconf
        self.updater.start()
        self.reconf.config_callback = self.config_callback
        self._node_grn = node_name

    def get_node_grn(self):

        return self._node_grn

    def config_callback(self, config):

        #TODO: Think about replacing callback architecture with signals.

        if config:
            # TODO: should use config.keys but this method doesnt exist

            names = [name for name, v in config.items()]
            # v isn't used but necessary to get key and put it into dict.
            rospy.logdebug('config_callback name={} v={}'.format(name, v))

            for widget in self.editor_widgets:
                if isinstance(widget, EditorWidget):
                    if widget.param_name in names:
                        rospy.logdebug('EDITOR widget.param_name=%s',
                                       widget.param_name)
                        widget.update_value(config[widget.param_name])
                elif isinstance(widget, GroupWidget):
                    cfg = find_cfg(config, widget.param_name)
                    rospy.logdebug('GROUP widget.param_name=%s',
                                   widget.param_name)
                    widget.update_group(cfg)

    def _handle_load_clicked(self):
        filename = QFileDialog.getOpenFileName(
                self, self.tr('Load from File'), '.',
                self.tr('YAML file {.yaml} (*.yaml)'))
        if filename[0] != '':
            self.load_param(filename[0])

    def _handle_save_clicked(self):
        filename = QFileDialog.getSaveFileName(
                self, self.tr('Save parameters to file...'), '.',
                self.tr('YAML files {.yaml} (*.yaml)'))
        if filename[0] != '':
            self.save_param(filename[0])

    def save_param(self, filename):
        configuration = self.reconf.get_configuration()
        if configuration is not None:
            with file(filename, 'w') as f:
                yaml.dump(configuration, f)

    def load_param(self, filename):
        with file(filename, 'r') as f:
            configuration = {}
            for doc in yaml.load_all(f.read()):
                configuration.update(doc)
        self.reconf.update_configuration(configuration)

    def close(self):
        self.reconf.close()
        self.updater.stop()

        for w in self.editor_widgets:
            w.close()

        self.deleteLater()

    def filter_param(self, filter_key):
        #TODO impl
        pass
