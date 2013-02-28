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

from python_qt_binding import loadUi
from python_qt_binding.QtCore import QModelIndex, QSize
from python_qt_binding.QtGui import (QDialog, QGridLayout, QLabel, QLineEdit,
                                     QPushButton, QStandardItem,
                                     QStandardItemModel, QStyle, QToolButton,
                                     QWidget)
import roslaunch
from roslaunch.core import RLException
import rospkg
import rospy

from rqt_launch.node_proxy import NodeProxy
from rqt_launch.node_controller import NodeController
#from rqt_launch.node_gui import NodeGui
from rqt_launch.node_delegate import NodeDelegate
from rqt_launch.status_indicator import StatusIndicator
from rqt_py_common.rqt_roscomm_util import RqtRoscommUtil


class LaunchWidget(QDialog):
    """
    #TODO: comment
    """

    def __init__(self, config, parent):
        """
        @type parent: LaunchMain
        @type config: ?
        """
        super(LaunchWidget, self).__init__()
        self._parent = parent
        self._config = config

        #TODO: should be configurable from gui
        self._port_roscore = 11311

        self._run_id = None
        rospy.loginfo(self._config.summary())
        # rospy.loginfo("MASTER", self._config.master.uri)  # Sheds error.
        #TODO: Replace 'print' with ROS-y method.
        print "MASTER", self._config.master.uri

        self._rospack = rospkg.RosPack()

        ui_file = os.path.join(self._rospack.get_path('rqt_launch'),
                               'resource', 'launch_widget.ui')
        loadUi(ui_file, self)

        self._datamodel = QStandardItemModel()
        #TODO: this layout is temporary. Need to be included in .ui.
        #self._gridlayout_process = None
        self._delegate = NodeDelegate(self._config.master.uri, self._rospack)
        self._treeview.setModel(self._datamodel)
        self._treeview.setItemDelegate(self._delegate)

        # NodeController used in controller class for launch operation.
        self._node_controllers = []

        self._pushbutton_start_stop_all.clicked.connect(self._parent.start_all)
        # Bind package selection with .launch file selection.
        self._combobox_pkg.currentIndexChanged[str].connect(
                                                 self._refresh_launchfiles)
        # Bind a launch file selection with launch GUI generation.
        self._combobox_launchfile_name.currentIndexChanged[str].connect(
                                                 self._load_launchfile_slot)
        self._update_pkgs_contain_launchfiles()

        # Used for removing previous nodes
        self._num_nodes_previous = 0

    def _load_launchfile_slot(self, launchfile_name):

        # Not yet sure why, but everytime combobox.currentIndexChanged occurs,
        # this method gets called twice with launchfile_name=='' in 1st call.
        if launchfile_name == None or launchfile_name == '':
            return

        _config = None

        rospy.loginfo('_load_launchfile_slot launchfile_name={}'.format(
                                                launchfile_name))

        try:
            _config = self._create_launchconfig(launchfile_name,
                                                self._port_roscore)
        except IndexError as e:
            #TODO: Show error msg on GUI
            rospy.logerr('IndexError={} launchfile_name={}'.format(
                                                e.message, launchfile_name))
            return
        except RLException as e:
            #TODO: Show error msg on GUI
            rospy.logerr('RLException={} launchfile_name={}'.format(
                                                e.message, launchfile_name))
            return

        #self._create_gui_for_launchfile(_config)
        self._create_widgets_for_launchfile(_config)

    def _create_launchconfig(self, launchfile_name, port_roscore=11311,
                             folder_name_launchfile='launch'):
        """
        @rtype: ROSLaunchConfig
        @raises IndexError:
        @raises RLException: raised by roslaunch.config.load_config_default.
        """

        #TODO: folder_name_launchfile foShould be able to specify arbitrarily.

        pkg_name = self._combobox_pkg.currentText()

        try:
            launchfile = os.path.join(self._rospack.get_path(pkg_name),
                                      folder_name_launchfile, launchfile_name)
        except IndexError as e:
            #TODO: Return exception to show error msg on GUI
            raise e

        try:
            launch_config = roslaunch.config.load_config_default([launchfile],
                                                                 port_roscore)
        except RLException as e:
            raise e

        return launch_config

    def _create_widgets_for_launchfile(self, config):
        self._config = config

        # Delete old nodes' GUIs.
        self._node_controllers = []

        # This seems to remove indexWidgets set on treeview.
        #TODO: Consider using beginResetModel() and endResetModel() as
        # suggested in API doc.
        # http://qt-project.org/doc/qt-4.8/qabstractitemmodel.html#reset
        self._datamodel.reset()

        # Need to store the num of nodes outside of the loop -- this will
        # be used for removing excessive previous node rows.
        row = 0
        for row, node in enumerate(self._config.nodes):  # Loop per node
            _proxy = NodeProxy(self._run_id, self._config.master.uri, node)

            # TODO: consider using QIcon.fromTheme()
            _status_label = StatusIndicator()

            #TODO: Ideally remove the next block.
            #BEGIN If these lines are missing, widget won't be shown either.
            std_item = QStandardItem()
            self._datamodel.setItem(row, 0, std_item)
            #END If these lines are missing, widget won't be shown either.

            qindex = self._datamodel.index(row, 0, QModelIndex())
            gui = self._delegate.create_node_widget(qindex, node,
                                                    _proxy.config,
                                                    _status_label)
            self._treeview.setIndexWidget(qindex, gui)

            node_controller = NodeController(_proxy, gui)
            self._node_controllers.append(node_controller)

            gui.connect_start_stop_button(node_controller.start)
            #stop_button.clicked.connect(node_controller.stop)

            rospy.loginfo('loop #%d _proxy.config.namespace=%s ' +
                          '_proxy.config.name=%s',
                          row, _proxy.config.namespace, _proxy.config.name)

        self._num_nodes_previous = row

        self._parent.set_node_controllers(self._node_controllers)

    def _update_pkgs_contain_launchfiles(self):
        """
        Inspired by rqt_msg.MessageWidget._update_pkgs_contain_launchfiles
        """
        packages = sorted([pkg_tuple[0]
                           for pkg_tuple
                           in RqtRoscommUtil.iterate_packages('launch')])
        self._package_list = packages
        rospy.loginfo('pkgs={}'.format(self._package_list))
        self._combobox_pkg.clear()
        self._combobox_pkg.addItems(self._package_list)
        self._combobox_pkg.setCurrentIndex(0)

    def _refresh_launchfiles(self, package=None):
        """
        Inspired by rqt_msg.MessageWidget._refresh_msgs
        """
        if package is None or len(package) == 0:
            return
        self._launchfile_instances = []  # Launch[]
        #TODO: RqtRoscommUtil.list_files's 2nd arg 'subdir' should NOT be
        # hardcoded later.
        _launch_instance_list = RqtRoscommUtil.list_files(package,
                                                         'launch')

        rospy.logdebug('_refresh_launches package={} instance_list={}'.format(
                                                       package,
                                                       _launch_instance_list))

        self._launchfile_instances = [x.split('/')[1]
                                      for x in _launch_instance_list]

        self._combobox_launchfile_name.clear()
        self._combobox_launchfile_name.addItems(self._launchfile_instances)

    def shutdown(self):
        # TODO: Needs implemented. Trigger dynamic_reconfigure to unlatch
        #            subscriber.
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # instance_settings.set_value('splitter', self._splitter.saveState())
        pass

    def restore_settings(self, plugin_settings, instance_settings):
#        if instance_settings.contains('splitter'):
#            self._splitter.restoreState(instance_settings.value('splitter'))
#        else:
#            self._splitter.setSizes([100, 100, 200])
        pass


if __name__ == '__main__':
    # main should be used only for debug purpose.
    # This launches this QWidget as a standalone rqt gui.
    from rqt_gui.main import Main

    main = Main()
    sys.exit(main.main(sys.argv, standalone='rqt_launch'))
