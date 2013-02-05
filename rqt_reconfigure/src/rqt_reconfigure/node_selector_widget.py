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
import os

import dynamic_reconfigure as dyn_reconf
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QRegExp, Qt, QTimer, Signal
from python_qt_binding.QtGui import QHeaderView, QItemSelectionModel, QStandardItemModel, QWidget
import rospkg
import rospy
import rosservice
from rqt_py_common.data_items import ReadonlyItem

from .filter_children_model import FilterChildrenModel
from .treenode_qstditem import TreenodeQstdItem
from .rqt_ros_graph import RqtRosGraph


class NodeSelectorWidget(QWidget):
    _COL_NAMES = ['Node']

    # public signal
    sig_node_selected = Signal(str)

    def __init__(self):
        super(NodeSelectorWidget, self).__init__()
        self.stretch = None

        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_reconfigure'), 'resource',
                               'node_selector.ui')
        loadUi(ui_file, self)

        # List of the available nodes. Since the list should be updated over
        # time and we don't want to create node instance per every update
        # cycle, This list instance should better be capable of keeping track.
        self._nodeitems = OrderedDict()
        # Dictionary. 1st elem is node's GRN name,
        # 2nd is TreenodeQstdItem instance.
        # TODO Needs updated when nodes list updated.

        #  Setup treeview and models
        self._std_model = QStandardItemModel()
        self._rootitem = self._std_model.invisibleRootItem()  # QStandardItem

        self._nodes_previous = None

        # Calling this method updates the list of the node.
        # Initially done only once.
        self._update_nodetree()

        # TODO(Isaac): Needs auto-update function enabled, once another
        #             function that updates node tree with maintaining
        #             collapse/expansion  state. http://goo.gl/GuwYp can be a
        #             help.

        self._collapse_button.pressed.connect(
                                          self._node_selector_view.collapseAll)
        self._expand_button.pressed.connect(self._node_selector_view.expandAll)

        # Filtering preparation.
        self._proxy_model = FilterChildrenModel(self)
        self._proxy_model.setDynamicSortFilter(True)
        self._proxy_model.setSourceModel(self._std_model)
        self._node_selector_view.setModel(self._proxy_model)
        self._filterkey_prev = ''

        # This 1 line is needed to enable horizontal scrollbar. This setting
        # isn't available in .ui file.
        # Ref. http://stackoverflow.com/a/6648906/577001
        self._node_selector_view.header().setResizeMode(
                                              0, QHeaderView.ResizeToContents)

        # Setting slot for when user clicks on QTreeView.
        self.selectionModel = self._node_selector_view.selectionModel()
        self.selectionModel.selectionChanged.connect(
                                                  self._selection_changed_slot)

    def _selection_deselected(self, index_current, rosnode_name_selected):
        """
        Intended to be called from _selection_changed_slot.
        """
        rospy.logdebug('_selection_deselected.')
        self.selectionModel.select(index_current, QItemSelectionModel.Deselect)
        self.sig_node_selected.emit(rosnode_name_selected)

    def _selection_selected(self, index_current, rosnode_name_selected):
        """
        Intended to be called from _selection_changed_slot.
        """
        rospy.logdebug('_selection_changed_slot row={} col={} data={}'.format(
                          index_current.row(), index_current.column(),
                          index_current.data(Qt.DisplayRole)))

        # Then determine if it's terminal treenode.
        found_node = False
        for n in self._nodeitems.itervalues():
            name = n.data(Qt.DisplayRole)
            name_sel = rosnode_name_selected[
                       rosnode_name_selected.rfind(RqtRosGraph.DELIM_GRN) + 1:]
            if ((name == rosnode_name_selected) and
                (name[name.rfind(RqtRosGraph.DELIM_GRN) + 1:] == name_sel)):
                rospy.logdebug('terminal str {} MATCH {}'.format(name,
                                                                 name_sel))
                found_node = True
                break
        if not found_node:  # Only when it's a terminal, move forward.
            self.selectionModel.select(index_current,
                                       QItemSelectionModel.Deselect)
            return

        item_child = self._std_model.itemFromIndex(index_current.child(0, 0))
        rospy.logdebug('item_selected={} item_child={} r={} c={}'.format(
                       index_current, item_child,
                       index_current.row(), index_current.column()))

        self.sig_node_selected.emit(rosnode_name_selected)

        # Show the node as selected.
        #selmodel.select(index_current, QItemSelectionModel.SelectCurrent)

    def _selection_changed_slot(self, selected, deselected):
        """
        Sends "open ROS Node box" signal ONLY IF the selected treenode is the
        terminal treenode.
        Receives args from signal QItemSelectionModel.selectionChanged.

        :param selected: All indexs where selected (could be multiple)
        :type selected: QItemSelection
        :type deselected: QItemSelection
        """

        ## Getting the index where user just selected. Should be single.
        if len(selected.indexes()) < 0 and len(deselected.indexes()) < 0:
            rospy.loginfo('Nothing selected? Not ideal to reach here')
            return

        if len(selected.indexes()) > 0:
            index_current = selected.indexes()[0]
        elif len(deselected.indexes()) == 1:
            # Setting length criteria as 1 is only a workaround, to avoid
            # Node boxes on right-hand side disappears when filter key doesn't
            # match them.
            # Indeed this workaround leaves another issue. Question for
            # permanent solution is asked here http://goo.gl/V4DT1
            index_current = deselected.indexes()[0]

        rosnode_name_selected = RqtRosGraph.get_upper_grn(index_current, '')
        if not rosnode_name_selected in self._nodeitems.keys():
            # De-select the selected item.
            self.selectionModel.select(index_current,
                                       QItemSelectionModel.Deselect)
            return

        if len(selected.indexes()) > 0:
            self._selection_selected(index_current, rosnode_name_selected)
        elif len(deselected.indexes()) > 0:
            self._selection_deselected(index_current, rosnode_name_selected)

    def _test_sel_index(self, selected, deselected):
        """
        Method for Debug only
        """
        #index_current = self.selectionModel.currentIndex()
        src_model = self._std_model
        index_current = None
        index_deselected = None
        index_parent = None
        curr_qstd_item = None
        if len(selected.indexes()) > 0:
            index_current = selected.indexes()[0]
            index_parent = index_current.parent()
            curr_qstd_item = src_model.itemFromIndex(index_current)
        elif len(deselected.indexes()) > 0:
            index_deselected = deselected.indexes()[0]
            index_parent = index_deselected.parent()
            curr_qstd_item = src_model.itemFromIndex(index_deselected)

        if len(selected.indexes()) > 0:
            rospy.logdebug('sel={} par={} desel={} sel.d={} par.d={}'.format(
                                 index_current, index_parent, index_deselected,
                                 index_current.data(Qt.DisplayRole),
                                 index_parent.data(Qt.DisplayRole),)
                                 + ' desel.d={} cur.item={}'.format(
                                 None,  # index_deselected.data(Qt.DisplayRole)
                                 curr_qstd_item))
        elif len(deselected.indexes()) > 0:
            rospy.logdebug('sel={} par={} desel={} sel.d={} par.d={}'.format(
                                 index_current, index_parent, index_deselected,
                                 None, index_parent.data(Qt.DisplayRole)) +
                           ' desel.d={} cur.item={}'.format(
                                 index_deselected.data(Qt.DisplayRole),
                                 curr_qstd_item))

    def get_paramitems(self):
        """
        :rtype: OrderedDict 1st elem is node's GRN name,
                2nd is TreenodeQstdItem instance
        """
        return self._nodeitems

    def _update_nodetree(self):
        """
        """

        # TODO(Isaac): 11/25/2012 dynamic_reconfigure only returns params that
        #             are associated with nodes. In order to handle independent
        #             params, different approach needs taken.
        try:
            nodes = dyn_reconf.find_reconfigure_services()
        except rosservice.ROSServiceIOException as e:
            rospy.logerr("Reconfigure GUI cannot connect to master.")
            raise e  # TODO Make sure 'raise' here returns or finalizes func.

        if not nodes == self._nodes_previous:
            i_node_curr = 1
            num_nodes = len(nodes)
            for node_name_grn in nodes:

                ####(Begin) For DEBUG ONLY; skip some dynreconf creation
#                if i_node_curr % 94 != 0:
#                    i_node_curr += 1
#                    continue
                #### (End) For DEBUG ONLY. ####

                # Please don't remove - this is not a debug print.
                rospy.loginfo('rqt_reconfigure loading #{}/{} node={}'.format(
                                        i_node_curr, num_nodes, node_name_grn))

                paramitem_full_nodename = TreenodeQstdItem(
                                 node_name_grn, TreenodeQstdItem.NODE_FULLPATH)
                #paramitem_full_nodename.set_param_name(node_name_grn)
                names = paramitem_full_nodename.get_param_names()

                # paramitem_full_nodename is the node that represents node.
                # self._nodeitems.append(paramitem_full_nodename)
                self._nodeitems[node_name_grn] = paramitem_full_nodename

                i_node_curr += 1
                rospy.logdebug('_update_nodetree i=%d names=%s',
                               i_node_curr, names)

                self._add_tree_node(paramitem_full_nodename,
                                    self._rootitem,
                                    names)

    def _add_tree_node(self, param_item_full,
                       stditem_parent, child_names_left):
        """
        Evaluate current treenode and the previous treenode at the same depth.
        If the name of both nodes is the same, current node instance is
        ignored. If not, the current node gets added to the same parent node.
        At the end, this function gets called recursively going down 1 level.

        :type param_item_full: TreenodeQstdItem
        :type stditem_parent: QStandardItem.
        :type child_names_left: List of str
        :param child_names_left: List of strings that is sorted in hierarchical
                                 order of params.
        """
        # TODO(Isaac): Consider moving to rqt_py_common.

        name_curr = child_names_left.pop(0)
        stditem_curr = TreenodeQstdItem(param_item_full.get_raw_param_name(),
                                        TreenodeQstdItem.NODE_FULLPATH)

        # item at the bottom is your most recent node.
        row_index_parent = stditem_parent.rowCount() - 1

        # Obtain and instantiate prev node in the same depth.
        name_prev = ''
        stditem_prev = None
        if not stditem_parent.child(row_index_parent) == None:
            stditem_prev = stditem_parent.child(row_index_parent)
            name_prev = stditem_prev.text()

        stditem = None
        if name_prev != name_curr:
            stditem_curr.setText(name_curr)
            stditem_parent.appendRow(stditem_curr)
            stditem = stditem_curr
        else:
            stditem = stditem_prev

        rospy.logdebug('add_tree_node 1 name_curr=%s ' +
                       '\n\t\t\t\t\tname_prev=%s row_index_parent=%d',
                       name_curr, name_prev, row_index_parent)

        if len(child_names_left) != 0:
            # TODO: View & Model are closely bound here. Ideally isolate this
            #       2. Maybe we should split into 2 classs, 1 handles view,
            #       the other does model.
            self._add_tree_node(param_item_full, stditem, child_names_left)

    def _refresh_nodes(self):
        # TODO(Isaac) In the future, do NOT remove all nodes. Instead,
        #            remove only the ones that are gone. And add new ones too.

        model = self._rootitem
        if model.hasChildren():
            row_count = model.rowCount()
            model.removeRows(0, row_count)
            rospy.logdebug("ParamWidget _refresh_nodes row_count=%s",
                           row_count)
        self._update_nodetree()

    def close_node(self):
        rospy.logdebug(" in close_node")
        # TODO(Isaac) Figure out if dynamic_reconfigure needs to be closed.

    def filter_key_changed(self, text):
        """
        Slot that accepts filtering key.

        Taken from example:
        http://doc.qt.digia.com/qt/itemviews-basicsortfiltermodel.html
        """

#        if '' == text:
#            # When filter key is empty, search with all text by regex.
#            #TODO However, this resets selections and collapse tree.
#            #Need improvement.
#            self._proxy_model.reset()
#            text = "[a-z0-9]*"

        if ((not text or text.isspace()) or  # Only space or 0-length strings.
            text == self._filterkey_prev):
            return
        else:
            self._filterkey_prev = text

        # Other than RegEx, Wild card, Fixed text are also possible. Right now
        # RegEx is in use in hope of it works the best.
        syntax_nr = QRegExp.RegExp

        rospy.loginfo('Filter key={}'.format(text))
        syntax = QRegExp.PatternSyntax(syntax_nr)
        regExp = QRegExp(text, Qt.CaseInsensitive, syntax)
        self._proxy_model.setFilterRegExp(regExp)
