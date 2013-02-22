#! /usr/bin/env python


class NodeGui(object):
    __slots__ = ['status_label', 'respawn_toggle', 'spawn_count_label',
                 'launch_prefix_edit']

    def __init__(self, status_label, respawn_toggle, spawn_count_label,
                 launch_prefix_edit):
        self.status_label = status_label
        self.respawn_toggle = respawn_toggle
        self.spawn_count_label = spawn_count_label
        self.launch_prefix_edit = launch_prefix_edit
