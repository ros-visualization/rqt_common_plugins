#! /usr/bin/env python


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
        @type proxy: NodeProxy
        @type gui: QItemDelegate
        """
        self._proxy = proxy

        self._gui = gui

    def start(self, restart=True):
        if self._proxy.is_running():
            if not restart:
                return
            self._gui.label_status.set_stopping()
            self._proxy.process.stop()

        # If the launch_prefix has changed, then the process must be re-created
        if (self._proxy.config.launch_prefix !=
            self._gui._lineEdit_launch_prefix.text()):

            self._proxy.config.launch_prefix = self._gui._lineEdit_launch_prefix.text()
            self._proxy.recreate_process()

        self._gui.label_status.set_starting()
        self._proxy.process.start()
        self._gui.label_status.set_running()
        self._gui.label_spawncount.setText("({})".format(
                                              self._proxy.process.spawn_count))

    def stop(self):
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
