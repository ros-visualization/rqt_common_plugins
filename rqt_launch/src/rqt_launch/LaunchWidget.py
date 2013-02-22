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

from __future__ import division

import sys

from python_qt_binding.QtCore import Signal
from python_qt_binding.QtGui import (QLabel, QHBoxLayout, QSplitter,
                                     QVBoxLayout, QWidget)


class LaunchWidget(QWidget):
    _TITLE_PLUGIN = 'Launch'

    def __init__(self, context, node=None):
        """

        """

        super(LaunchWidget, self).__init__()
        self.setObjectName(self._TITLE_PLUGIN)
        self.setWindowTitle(self._TITLE_PLUGIN)

        if node is None:
            title = self._TITLE_PLUGIN
        else:
            title = self._TITLE_PLUGIN + ' %s' % node
        self.setObjectName(title)

    def shutdown(self):
        #TODO: Needs implemented. Trigger dynamic_reconfigure to unlatch
        #            subscriber.
        pass

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value('splitter', self._splitter.saveState())

    def restore_settings(self, plugin_settings, instance_settings):
        if instance_settings.contains('splitter'):
            self._splitter.restoreState(instance_settings.value('splitter'))
        else:
            self._splitter.setSizes([100, 100, 200])


if __name__ == '__main__':
    # main should be used only for debug purpose.
    # This launches this QWidget as a standalone rqt gui.
    from rqt_gui.main import Main

    main = Main()
    sys.exit(main.main(sys.argv, standalone='rqt_launch'))
