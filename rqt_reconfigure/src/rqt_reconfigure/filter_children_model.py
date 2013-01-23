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

from collections import OrderedDict

import dynamic_reconfigure.client
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QSortFilterProxyModel
import rospy

#from .rqt_ros_graph import RqtRosGraph
#from .treenode_status import TreenodeStatus

class FilterChildrenModel(QSortFilterProxyModel):

    def __init__(self, parent=None):
        super(FilterChildrenModel, self).__init__(parent)
        
        # :Key: Internal ID of QModelIndex of each treenode.
        # :Value: TreenodeStatus 
        #self._treenodes = OrderedDict()

        self._toplv_parent_prev = None
        
    def filterAcceptsRow(self, src_row, src_parent_qmindex):
        """
        Overridden.
        
        Terminology:
        "Treenode" is deliberately used to avoid confusion with "Node" in ROS.
        
        :type src_row: int
        :type src_parent_qmindex: QModelIndex
        """
        
        return self._filter_row(src_row, src_parent_qmindex)
      
    def filterAcceptsColumn(self, source_column, source_parent):
        """
        Overridden.
        
        :type source_column: int
        :type source_parent: QModelIndex
        """
        rospy.logdebug('FCModel.filterAcceptsCol source_col={} '.format(
            source_column) + 'parent col={} row={} data={}'.format(
            source_parent.column(), source_parent.row(), source_parent.data()))
        return True
    
    def _filter_row(self, src_row, src_parent_qmindex):
        curr_qmindex = self.sourceModel().index(src_row, 0, src_parent_qmindex)
        curr_qstd_item = self.sourceModel().itemFromIndex(curr_qmindex)
        #parent_item = curr_qmindex.internalPointer()        
        rospy.logdebug('Nodename full={} Top treenode={}'.format(
           curr_qstd_item.get_raw_param_name(), curr_qstd_item.get_node_name()))
        nodename_fullpath = curr_qstd_item.get_raw_param_name()

        regex = self.filterRegExp()        
        pos_hit = regex.indexIn(nodename_fullpath)
        if pos_hit >= 0:  # Query hit.
            # Set all subsequent treenodes True
            rospy.logdebug('  FCModel.filterAcceptsRow src_row={}'.format(src_row) +
                          ' parent row={} data={}'.format(
                              src_parent_qmindex.row(),
                              src_parent_qmindex.data()) +
                          ' filterRegExp={}'.format(regex))
            
            # Once we find a treenode that hits the query, no need to further
            # traverse since what this method wants to know with the given
            # index is whether the given index is supposed to be shown or not. 
            # Thus, just return True here.
            return True
        
        # Evaluate children recursively.
        row_child = 0
        while True:
            child_qmindex = curr_qmindex.child(row_child, 0)
            if child_qmindex.isValid():
                flag = self._filter_row(row_child, curr_qmindex)
                if flag:
                    return True
            else:
                return False
            row_child += 1
        
        return False
         
    def _filter_row_precedent(self, src_row, src_parent_qmindex):
        """
        Check treenodes in top-down direction.
        Bottom-up direction is impossible due to QSortFilterProxyModel's 
        design/limitation. 
        """
        curr_qmindex = self.sourceModel().index(src_row, 0, src_parent_qmindex)
        
        # Generate Treenode instance
        curr_id = curr_qmindex.internalId()
        curr_nodename_segmented = str(curr_qmindex.data())
        if self._treenodes.has_key(curr_id):
            curr_treenode = self._treenodes.get(curr_id)           

            if not curr_qmindex.internalId == self._toplv_parent_prev.internalId:
                curr_treenode.set_is_eval_done(True)
            else:
                curr_treenode.set_is_eval_done(False) # Reset treenode instance.

        else:
            self._treenodes[curr_id](TreenodeStatus(curr_id))
            self._toplv_parent_prev = self._get_toplevel_parent_recur(curr_qmindex)
            

        flag = False
        # From curr_qmindex (QModelIndex), obtain terminal item that also
        # contains full path node name.
        node_names = RqtRosGraph.get_full_grn_recur(curr_qmindex, False)
        rospy.logdebug('Nodes len={}'.format(len(node_names)))

        #TODO Add creation of all subsequent treenodes. Only occurs when 
        #     the current qmindex is top level treenode.
#        if not src_parent_qmindex == -1:
#            self._gen_children_treenodes(curr_id)

        regex = self.filterRegExp()
        i_debug = 0
        # Loop per all sub children full path nodes.
        for node_name_selected in node_names:
            rospy.loginfo('i={} {}'.format(i_debug, node_name_selected))
            #regex = self.filterRegExp()
            i = regex.indexIn(node_name_selected)
            if i >= 0:  # Query hit.
                flag = True
                # Set all subsequent treenodes True
                rospy.logdebug('  FCModel.filterAcceptsRow src_row={}'.format(src_row) +
                              ' parent row={} data={}'.format(
                                  src_parent_qmindex.row(),
                                  src_parent_qmindex.data()) +
                              ' filterRegExp={}'.format(regex))
            else:  # Query didn't hit.
                #flag = False

                # Turn the TreenodeStatus' of all parents _shows flag to False
                rospy.loginfo(" xxxxxx query didn't match node={}".format(node_name_selected))
            i_debug += 1
        return flag        

    def _get_toplevel_parent_recur(self, qmindex):
        p = qmindex.parent()
        if p.isValid():
            self._get_toplevel_parent(p)
        return p
    
    def _gen_children_treenodes(self, qmindex_curr):
        """
        :rtype" 
        """
        i_child = 0
        children_dict = OrderedDict()
        while True: # Loop per child.
            grn_curr = grn_prev + DELIM_GRN + str(qmindex_curr.data())
            child_qmindex = qmindex_curr.child(i_child, 0)
            
            if (not child_qmindex.isValid()):
                rospy.logdebug('!! DEADEND i_child=#{} grn_curr={}'.format(i_child,
                                                                          grn_curr))
                if i_child == 0:
                    # Only when the current node has no children, add current 
                    # GRN to the returning list.
                    children_dict.append(grn_curr)
                return children_dict

            list_grn_children = self._gen_children_treenodes(child_qmindex)

            for child_grn in list_grn_children:
                child_grn = grn_prev + (DELIM_GRN + grn_curr) + (DELIM_GRN + child_grn)
                
            children_dict = children_dict + list_grn_children
            rospy.logdebug('111 lennodes={} list_grn_children={}'.format(
                                len(children_dict), list_grn_children))
            rospy.logdebug('122 children_dict={}'.format(children_dict))
            i_child += 1
        return children_dict