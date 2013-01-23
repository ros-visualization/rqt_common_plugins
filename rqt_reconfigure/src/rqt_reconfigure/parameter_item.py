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

import dynamic_reconfigure.client
from python_qt_binding.QtCore import Qt, QVariant
from python_qt_binding.QtGui import QWidget
import rospy
from rqt_py_common.data_items import ReadonlyItem

from .dynreconf_client_widget import DynreconfClientWidget

class ParameterItem(ReadonlyItem):
    """   
    Extending ReadonlyItem - the display content of this item shouldn't be 
    modified.
    """
    
    NODE_FULLPATH = 1
            
    def __init__(self, *args):
        """
        :param args: 1st elem: str (will become 1st arg of QStandardItem) 
                     2nd elem: integer value that indicates whether this class 
                               is node that has GRN (Graph Resource Names, see 
                               http://www.ros.org/wiki/Names). This can be None.
        """
        #super(ParameterItem, self).__init__(*args)
        treenode_name = args[0]
        self._param_name_raw = treenode_name
        self._set_param_name(treenode_name) #self._nodename = treenode_name
        super(ParameterItem, self).__init__(treenode_name)
        
        self.node_full = False
        
        try:
            if args[1] != None:
                self.node_full = True
                rospy.logdebug('ParameterItem now loading node={}'.format(treenode_name))
                self._dynreconf_client = self._create_paramclient(treenode_name) 
        except IndexError as e: #tuple index out of range etc.
            rospy.logdebug('ParameterItem fullpath=F')
            
    def get_widget(self):
        """
        :rtype: DynreconfClientWidget (QWidget)
        """
        return self._dynreconf_client
        
    def _create_paramclient(self, nodename):
        """        
        Callback when user chooses a node.
        
        :param nodename: GRN (Graph Resource Names, 
                         see http://www.ros.org/wiki/Names) of node name.
        :type node: str
        :rtype: DynreconfClientWidget
        """        
        try:
            _dynreconf_client = dynamic_reconfigure.client.Client(str(nodename),
                                                                  timeout=5.0)
        except rospy.exceptions.ROSException:
            rospy.logerr("ParameterItem. Could not connect to node {}".format(
                                                                     nodename))
            #TODO(Isaac) Needs to show err msg on GUI too. 
            return
#        finally:
#            if self._dynreconf_client:
#                self._dynreconf_client.close() #Close old GUI client.

        dynreconf_widget = DynreconfClientWidget(_dynreconf_client, nodename)
        return dynreconf_widget 
    
    def _set_param_name(self, param_name):
        """
        :param param_name: A string formatted as GRN (Graph Resource Names, see  
                           http://www.ros.org/wiki/Names). 
                           Example: /paramname/subpara/subsubpara/...
        """       
        #  separate param_name by forward slash
        self._list_paramname = param_name.split('/')
        
        #  Deleting the 1st elem which is zero-length str.
        del self._list_paramname[0]  
        
        self._nodename = self._list_paramname[0]
        
        rospy.logdebug('_set_param_name param_name={}  self._nodename={}  self._list_paramname[-1]={}'.format(
                       param_name, self._nodename, self._list_paramname[-1]))
                        
    def get_param_name_toplv(self):
        """
        :rtype: String of the top level param name.
        """ 

        return self._name_top
    
    def get_raw_param_name(self):
        return self._param_name_raw
    
    def get_param_names(self):
        """
        :rtype: List of string. Null if param
        """
    
        #TODO what if self._list_paramname is empty or null?
        return self._list_paramname

    def get_node_name(self):
        """
        :return: A value of single tree node (ie. NOT the fullpath node name).
                 Ex. suppose fullpath name is /top/sub/subsub/subsubsub and you
                     are at 2nd from top, the return value is subsub. 
        """
        return self._nodename
        
    def type(self):
        return QStandardItem.UserType
