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

import os

import dynamic_reconfigure.client
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QStandardItemModel, QVBoxLayout, QWidget
import rospkg
import rospy

from .dynreconf_client_widget import DynreconfClientWidget
from .node_delegate import NodeDelegate
from .param_editors import EditorWidget, BooleanEditor, DoubleEditor, EnumEditor, IntegerEditor, StringEditor
from .param_groups import GroupWidget
from .param_updater import ParamUpdater

class ParameditWidget(QWidget):
    """
    This class represents a pane where parameter editor widgets of multiple nodes
    are shown. In rqt_reconfigure, this pane occupies right half of the entire 
    visible area.    
    """
    
    def __init__(self, paramitems_dict):
        """
        :type paramitems_dict: OrderedDict. 1st elem is node's GRN name,
                          2nd is ParameterItem instance (that extends 
                          QStandardItem)
        """
        super(ParameditWidget, self).__init__()
        
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_reconfigure'), 
                               'resource', 'paramedit_pane.ui')
        loadUi(ui_file, self)
        
        self._dynreconf_clients = []
        self._nodenames_displayed = []
       
        #self.node_delegate = NodeDelegate(self, paramitems_dict)
        #self.listview.setItemDelegate(self.node_delegate)

        #self.set_nodes(paramitems_dict)
        #self.std_model = QStandardItemModel()
        #self.listview.setModel(self.std_model) # QListView
        
        # Adding the list of Items 
        #self.std_model.insertColumn(0, paramitems_dict.values())
        self.vlayout = QVBoxLayout(self.scrollarea_holder_widget)
        
        # Alternate background color 
#        count_node = 0
#        for v in paramitems_dict.itervalues():
#            w = v.get_widget()
#            item = self.std_model.item(count_node, 0)
#            if count_node % 2 != 0:
#                w.setAutoFillBackground(True)
#                p = w.palette()
#                p.setColor(w.backgroundRole(), Qt.gray)
#                w.setPalette(p)
#
#            self.listview.setIndexWidget(item.index(), w)
##            self.vlayout.addWidget(w)
#            count_node += 1
        
        #self._set_index_widgets(self.listview, paramitems_dict) #causes error        

        self.destroyed.connect(self.close)  # func in mercurial?
        
    def _set_index_widgets(self, view, paramitems_dict):
        '''
        :deprecated: Causes error  
        '''
        i = 0
        for p in paramitems_dict:
            view.setIndexWidget(i, p)
            i += 1
        
    def show_reconf(self, node_grn):
        """        
        Callback when user chooses a node.
        
        :deprecated: move_to_node should be used due to the enhancement
                     request https://github.com/ros-visualization/rqt_common_plugins/issues/4
        :param node_grn: GRN (Graph Resource Names, see http://www.ros.org/wiki/Names)  
                     of node name.
        :type node_grn: str
        """
        rospy.logdebug('ParameditWidget.show str(node_grn)=%s', str(node_grn))

        dynreconf_client = None
        try:
            dynreconf_client = dynamic_reconfigure.client.Client(str(node_grn),
                                                                 timeout=5.0)
        except rospy.exceptions.ROSException:
            rospy.logerr("Could not connect to %s" % node_grn)
            #TODO(Isaac) Needs to show err msg on GUI too. 
            return
         # Comment these lines out, since closing dyn_reconf client
         # doesn't make sense now that multiple clients can exits simultaneously.
         # TODO (Isaac) Better to figure out why closing at this point.
#        finally: 
#            if self._dynreconf_client:
#                self._dynreconf_client.close() #Close old GUI client.

        _dynreconf_client = DynreconfClientWidget(dynreconf_client, node_grn)
        # Client gets renewed every time different node_grn was clicked.

        #TODO Commented in.
        #self._paramedit_scrollarea.setWidget(self._dynreconf_client)
        #self._paramedit_scrollarea.setWidgetResizable(True)
        if not node_grn in self._nodenames_displayed:
            self._nodenames_displayed.append(node_grn)
            self._dynreconf_clients.append(_dynreconf_client)
            self.vlayout.addWidget(_dynreconf_client)

    def close(self):
        for dc in self._dynreconf_clients:
            # Clear out the old widget
            dc.close()
            dc = None

            self._paramedit_scrollarea.deleteLater()

    def set_nodes(self, nodeitems):
        """
        :type nodeitems: ParameterItem[] (that extends QStandardItem)
        """
        
        self.node_delegate.set_nodeitems(nodeitems)
             
        #TODO Add EditorWidgets
        #     setIndexWidget() might be it. 
        #     http://doc.qt.digia.com/qt/qtableview.html
                
        self.std_model.insertColumn(0, nodeitems)
        self.listview.setModel(self.std_model) # QListView
    
    def filter_param(self, filter_key):
        """
        :type filter_key:
        """
        
        #TODO Pick nodes that match filter_key.
        
        #TODO For the nodes that are kept in previous step, call 
        #     DynreconfWidget.filter_param for all of its existing 
        #     instances. 
        pass

    def move_to_node(self, node):
        """
        Move the visible region to the widget of correspondent node group.
        
        :since : 1/4/2013
        """
        rospy.logdebug('ParameditWidget.move_to_node str(node)=%s', str(node))
        
        #TODO Figure out the row index from node name.
         
        #TODO Move cursor to the corresponding node. Use QTableView.selectRow
