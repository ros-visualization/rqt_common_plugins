# Software License Agreement (BSD License)
#
# Copyright (c) 2016, Ryohei Ueda
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

import rosgraph
import rosnode
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QCheckBox, QScrollArea, QPushButton


class NodeSelection(QWidget):
    def __init__(self, parent):
        super(NodeSelection, self).__init__()
        self.parent_widget = parent
        self.selected_nodes = []
        self.setWindowTitle("Select the nodes you want to record")
        self.resize(500, 700)
        self.area = QScrollArea(self)
        self.main_widget = QWidget(self.area)
        self.ok_button = QPushButton("Done", self)
        self.ok_button.clicked.connect(self.onButtonClicked)
        self.ok_button.setEnabled(False)
        self.main_vlayout = QVBoxLayout(self)
        self.main_vlayout.addWidget(self.area)
        self.main_vlayout.addWidget(self.ok_button)
        self.setLayout(self.main_vlayout)

        self.selection_vlayout = QVBoxLayout(self)

        self.node_list = rosnode.get_node_names()
        self.node_list.sort()
        for node in self.node_list:
            self.addCheckBox(node)
        self.main_widget.setLayout(self.selection_vlayout)
        self.show()

    def addCheckBox(self, node):
        item = QCheckBox(node, self)
        item.stateChanged.connect(lambda x: self.updateNode(x, node))
        self.selection_vlayout.addWidget(item)

    def updateNode(self, state, node):
        if state == Qt.Checked:
            self.selected_nodes.append(node)
        else:
            self.selected_nodes.remove(node)
        if len(self.selected_nodes) > 0:
            self.ok_button.setEnabled(True)
        else:
            self.ok_button.setEnabled(False)

    def onButtonClicked(self):
        master = rosgraph.Master('rqt_bag_recorder')
        state = master.getSystemState()
        subs = [t for t, l in state[1]
                if len([node_name for node_name in self.selected_nodes if node_name in l]) > 0]
        for topic in subs:
            self.parent_widget.changeTopicCheckState(topic, Qt.Checked)
            self.parent_widget.updateList(Qt.Checked, topic)
        self.close()
