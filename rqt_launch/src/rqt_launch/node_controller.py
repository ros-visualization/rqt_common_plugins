#! /usr/bin/env python

import rospy


# Provides callback functions for the start and stop buttons
class NodeController(object):
    """
    Containing both proxy and gui instances, this class gives a control of
    a node on both ROS & GUI sides.
    """

    # these values need to synch with member variables.
    # Eg. self.gui isn't legal.
    __slots__ = ['_proxy', '_gui']

    def __init__(self, proxy, gui):
        """
        @type proxy: rqt_launch.NodeProxy
        @type gui: QWidget
        """
        self._proxy = proxy

        self._gui = gui

    def start_stop_slot(self, signal):
        """
        Works as a slot particularly intended to work for
        QAbstractButton::toggled(checked). Internally calls
        NodeController.start / stop depending on `signal`.

        @type signal: bool
        """
        if signal:
            self.start()
        else:
            self.stop()

    def start(self, restart=True):
        """
        Start a ROS node as a new process.
        """
        rospy.logdebug('Controller.start restart={}'.format(restart))

        if self._proxy.is_running():
            if not restart:
                return
            self._gui.label_status.set_stopping()
            self._proxy.process.stop()

        # If the launch_prefix has changed, then the process must be re-created
        if (self._proxy.config.launch_prefix !=
            self._gui._lineEdit_launch_prefix.text()):

            self._proxy.config.launch_prefix = \
                                     self._gui._lineEdit_launch_prefix.text()
            self._proxy.recreate_process()

        self._gui.label_status.set_starting()
        self._proxy.process.start()
        self._gui.label_status.set_running()
        self._gui.label_spawncount.setText("({})".format(
                                              self._proxy.process.spawn_count))

    def stop(self):
        """
        Stop a ROS node's process.
        """
        if self._proxy.is_running():
            self._gui.label_status.set_stopping()
            self._proxy.process.stop()
            self._gui.label_status.set_stopped()

    def check_process_status(self):
        if self._proxy.has_died():
            print "Process died: %s" % self._proxy.process.name
            self._proxy.process.stop()
            if self._proxy.process.exit_code == 0:
                self._gui.label_status.set_stopped()
            else:
                self._gui.label_status.set_died()

            # Checks if it should be respawned
            if self._gui.respawn_toggle.isChecked():
                print "Respawning process: %s" % self._proxy.process.name
                self._gui.label_status.set_starting()
                self._proxy.process.start()
                self._gui.label_status.set_running()
                self._gui.label_spawncount.setText("({})".format(
                                              self._proxy.process.spawn_count))

    def get_node_widget(self):
        """
        @rtype: QWidget
        """
        return self._gui
