#! /usr/bin/env python

import sys, os, os.path
import signal
import time
import uuid
import logging

from python_qt_binding import QtCore, QtGui
import roslib
from ros import roslaunch
import rospy


def handle_sigint(*args):
    sys.stderr.write("\rSIGINT")
    QtGui.QApplication.quit()


class StatusIndicator(QtGui.QLabel):
    def __init__(self, *args):
        super(StatusIndicator, self).__init__(*args)
        self.set_stopped()

    def set_running(self):
        self.setPixmap(self.style().standardIcon(QtGui.QStyle.SP_DialogApplyButton).pixmap(16))

    def set_starting(self):
        self.setPixmap(self.style().standardIcon(QtGui.QStyle.SP_DialogResetButton).pixmap(16))

    def set_stopping(self):
        self.setPixmap(self.style().standardIcon(QtGui.QStyle.SP_DialogResetButton).pixmap(16))

    def set_stopped(self):
        self.setText(" ")

    def set_died(self):
        self.setPixmap(self.style().standardIcon(QtGui.QStyle.SP_MessageBoxCritical).pixmap(16))


class NodeGui(object):
    __slots__ = ['status_label', 'respawn_toggle', 'spawn_count_label', 'launch_prefix_edit']

    def __init__(self, status_label, respawn_toggle, spawn_count_label, launch_prefix_edit):
        self.status_label = status_label
        self.respawn_toggle = respawn_toggle
        self.spawn_count_label = spawn_count_label
        self.launch_prefix_edit = launch_prefix_edit

# Provides callback functions for the start and stop buttons
class NodeController(object):
    __slots__ = ['proxy', 'gui']

    def __init__(self, proxy, gui):
        self.proxy = proxy
        self.gui = gui

    def start(self, restart=True):
        if self.proxy.is_running():
            if not restart:
                return
            self.gui.status_label.set_stopping()
            self.proxy.process.stop()

        # If the launch_prefix has changed, then the process must be re-created
        if self.proxy.config.launch_prefix != self.gui.launch_prefix_edit.text():
            self.proxy.config.launch_prefix = self.gui.launch_prefix_edit.text()
            self.proxy.recreate_process()
            # self.proxy.process = roslaunch.nodeprocess.create_node_process(self.run_id, self.config, self.master_uri)


        self.gui.status_label.set_starting()
        self.proxy.process.start()
        self.gui.status_label.set_running()
        self.gui.spawn_count_label.setText("(%d)" % self.proxy.process.spawn_count)

    def stop(self):
        if self.proxy.is_running():
            self.gui.status_label.set_stopping()
            self.proxy.process.stop()
            self.gui.status_label.set_stopped()


    def check_process_status(self):
        if self.proxy.has_died():
            print "Process died: %s" % self.proxy.process.name
            self.proxy.process.stop()
            if self.proxy.process.exit_code == 0:
                self.gui.status_label.set_stopped()
            else:
                self.gui.status_label.set_died()

            # Checks if it should be respawned
            if self.gui.respawn_toggle.isChecked():
                print "Respawning process: %s" % self.proxy.process.name
                self.gui.status_label.set_starting()
                self.proxy.process.start()
                self.gui.status_label.set_running()
                self.gui.spawn_count_label.setText("(%d)" % self.proxy.process.spawn_count)

class NamesSurrogate(object):
    """
    Because some functions in roslib.names cannot be referred in the original rxlaunch code, 
    the codes of those function are copied here. This class should not be used for 
    any other purpose than to be used within this .py file.

    :author: Isaac Saito
    """

    PRIV_NAME = '~'
    SEP = '/'

    @staticmethod
    def is_global(name):
        """
        Test if name is a global graph resource name. 116 117 @param name: must be a legal name in canonical form 118 @type name: str 119 @return: True if name is a globally referenced name (i.e. /ns/name) 120 @rtype: bool
        """
        return name and name[0] == NamesSurrogate.SEP

    @staticmethod
    def is_private(name):
        """ 126 Test if name is a private graph resource name. 127 128 @param name: must be a legal name in canonical form 129 @type name: str 130 @return bool: True if name is a privately referenced name (i.e. ~name) 131 """
        return name and name[0] == NamesSurrogate.PRIV_NAME

    @staticmethod
    def ns_join(ns, name):
        """ 
        Taken from http://ros.org/rosdoclite/groovy/api/roslib/html/python/roslib.names-pysrc.html#ns_join 
        since roslib.names is not found for some reason, and also the entire module seems deprecated.

        Join a namespace and name. If name is unjoinable (i.e. ~private or 162 /global) it will be returned without joining 163 164 @param ns: namespace ('/' and '~' are both legal). If ns is the empty string, name will be returned. 165 @type ns: str 166 @param name str: a legal name 167 @return str: name concatenated to ns, or name if it is 168 unjoinable. 169 @rtype: str 170 
        """
        if NamesSurrogate.is_private(name) or NamesSurrogate.is_global(name):
            return name
        if ns == NamesSurrogate.PRIV_NAME:
            return NamesSurrogate.PRIV_NAME + name
        if not ns:
            return name
        if ns[-1] == NamesSurrogate.SEP:
            return ns + name
        return ns + NamesSurrogate.SEP + name



def main():
    app = QtGui.QApplication(sys.argv)

    # Sets up signal handling so SIGINT closes the application,
    # following the solution given at [1].  Sets up a custom signal
    # handler, and ensures that the Python interpreter runs
    # occasionally so the signal is handled.  The email thread at [2]
    # explains why this is necessary.
    #
    # [1] http://stackoverflow.com/questions/4938723/#4939113
    # [2] http://www.mail-archive.com/pyqt@riverbankcomputing.com/msg13757.html
    signal.signal(signal.SIGINT, handle_sigint)
    timer = QtCore.QTimer()
    timer.start(250)
    timer.timeout.connect(lambda: None)  # Forces the interpreter to run every 250ms

    form = RxlaunchApp(sys.argv)
    status_checker_timer = QtCore.QTimer()
    status_checker_timer.timeout.connect(form.check_process_statuses)
    status_checker_timer.start(100)
    form.show()

    exit_code = -1
    try:
        exit_code = app.exec_()
    finally:
        form.stop_all()
    sys.exit(exit_code)


if __name__ == '__main__': main()
