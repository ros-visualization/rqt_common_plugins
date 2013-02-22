#! /usr/bin/env python

# Provides callback functions for the start and stop buttons
class NodeController(object):
    __slots__ = ['_proxy', 'gui']

    def __init__(self, proxy, gui):
        self._proxy = proxy
        self.gui = gui

    def start(self, restart=True):
        if self._proxy.is_running():
            if not restart:
                return
            self.gui.status_label.set_stopping()
            self._proxy.process.stop()

        # If the launch_prefix has changed, then the process must be re-created
        if self._proxy.config.launch_prefix != self.gui.launch_prefix_edit.text():
            self._proxy.config.launch_prefix = self.gui.launch_prefix_edit.text()
            self._proxy.recreate_process()

        self.gui.status_label.set_starting()
        self._proxy.process.start()
        self.gui.status_label.set_running()
        self.gui.spawn_count_label.setText("(%d)" % self._proxy.process.spawn_count)

    def stop(self):
        if self._proxy.is_running():
            self.gui.status_label.set_stopping()
            self._proxy.process.stop()
            self.gui.status_label.set_stopped()

    def check_process_status(self):
        if self._proxy.has_died():
            print "Process died: %s" % self._proxy.process.name
            self._proxy.process.stop()
            if self._proxy.process.exit_code == 0:
                self.gui.status_label.set_stopped()
            else:
                self.gui.status_label.set_died()

            # Checks if it should be respawned
            if self.gui.respawn_toggle.isChecked():
                print "Respawning process: %s" % self._proxy.process.name
                self.gui.status_label.set_starting()
                self._proxy.process.start()
                self.gui.status_label.set_running()
                self.gui.spawn_count_label.setText("(%d)" % self._proxy.process.spawn_count)

