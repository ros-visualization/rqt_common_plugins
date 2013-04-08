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

import math
import os

from python_qt_binding import loadUi
from python_qt_binding.QtGui import (QDoubleValidator, QIntValidator, QLabel,
                                     QWidget)
import rospkg
import rospy

EDITOR_TYPES = {
    'bool': 'BooleanEditor',
    'str': 'StringEditor',
    'int': 'IntegerEditor',
    'double': 'DoubleEditor',
}

# These .ui files are frequently loaded multiple times. Since file access
# costs a lot, only load each file once.
rp = rospkg.RosPack()
ui_bool = os.path.join(rp.get_path('rqt_reconfigure'), 'resource',
                       'editor_bool.ui')
ui_str = os.path.join(rp.get_path('rqt_reconfigure'), 'resource',
                      'editor_string.ui')
ui_num = os.path.join(rp.get_path('rqt_reconfigure'), 'resource',
                      'editor_number.ui')
ui_int = ui_num
ui_enum = os.path.join(rp.get_path('rqt_reconfigure'), 'resource',
                       'editor_enum.ui')


class EditorWidget(QWidget):

    def __init__(self, updater, config):
        """
        :param updater:
        :type updater: rqt_reconfigure.ParamUpdater
        """

        super(EditorWidget, self).__init__()

        self.updater = updater
        self.param_name = config['name']

        self.old_value = None

    def _update(self, value):
        if value != self.old_value:
            self.update_configuration(value)
            self.old_value = value

    def update_value(self, value):
        """
        To be overridden in subclass.

        Update the value of the arbitrary components based on user's input.
        """
        pass

    def update_configuration(self, value):
        self.updater.update({self.param_name: value})

    def display(self, grid):
        """
        Should be overridden in subclass.

        :type grid: QFormLayout
        :type row: ???
        """
        self._paramname_label.setText(self.param_name)
#        label_paramname = QLabel(self.param_name)
#        label_paramname.setWordWrap(True)
        self._paramname_label.setMinimumWidth(100)
        grid.addRow(self._paramname_label, self)

    def close(self):
        """
        Should be overridden in subclass.
        """
        pass


class BooleanEditor(EditorWidget):
    def __init__(self, updater, config):
        super(BooleanEditor, self).__init__(updater, config)
        loadUi(ui_bool, self)

        self.update_value(config['default'])
        self._checkbox.clicked.connect(self._update)

    def update_value(self, value):
        self._checkbox.setChecked(value)


class StringEditor(EditorWidget):
    def __init__(self, updater, config):
        super(StringEditor, self).__init__(updater, config)
        loadUi(ui_str, self)

        self._paramval_lineedit.editingFinished.connect(self.edit_finished)

    def update_value(self, value):
        rospy.logdebug('StringEditor update_value={}'.format(value))
        self._paramval_lineedit.setText(value)

    def edit_finished(self):
        rospy.logdebug('StringEditor edit_finished val={}'.format(
                                              self._paramval_lineedit.text()))
        self._update(self._paramval_lineedit.text())


class IntegerEditor(EditorWidget):
    def __init__(self, updater, config):
        super(IntegerEditor, self).__init__(updater, config)

        loadUi(ui_int, self)

        self.min = int(config['min'])
        self.max = int(config['max'])
        self._min_val_label.setText(str(self.min))
        self._max_val_label.setText(str(self.max))

        self._slider_horizontal.setRange(self.min, self.max)
        self._slider_horizontal.sliderReleased.connect(self.slider_released)
        self._slider_horizontal.sliderMoved.connect(self.update_text)
        # valueChanged gets called when groove is clicked on.
        self._slider_horizontal.valueChanged.connect(self.update_value)

        # TODO: Fix that the naming of _paramval_lineEdit instance is not
        #       consistent among Editor's subclasses.
        self._paramval_lineEdit.setValidator(QIntValidator(self.min,
                                                           self.max, self))
        self._paramval_lineEdit.editingFinished.connect(self.editing_finished)

        # TODO: This should not always get set to the default it should be the
        # current value
        self._paramval_lineEdit.setText(str(config['default']))
        self._slider_horizontal.setSliderPosition(int(config['default']))

    def slider_released(self):
        self.update_text(self._slider_horizontal.value())
        self._update(self._slider_horizontal.value())

    def update_text(self, val):
        rospy.logdebug(' IntegerEditor.update_text val=%s', str(val))
        self._paramval_lineEdit.setText(str(val))

    def editing_finished(self):
        self._slider_horizontal.setSliderPosition(
                                         int(self._paramval_lineEdit.text()))
        self._update(int(self._paramval_lineEdit.text()))

    def update_value(self, val):
        self._slider_horizontal.setSliderPosition(int(val))
        self._paramval_lineEdit.setText(str(val))
        rospy.logdebug(' IntegerEditor.update_val val=%s', str(val))


class DoubleEditor(EditorWidget):
    def __init__(self, updater, config):
        super(DoubleEditor, self).__init__(updater, config)

        loadUi(ui_num, self)

        # Handle unbounded doubles nicely
        if config['min'] != -float('inf'):
            self.min = float(config['min'])
            self._min_val_label.setText(str(self.min))
            self.func = lambda x: x
            self.ifunc = self.func
        else:
            self.min = -1e10000
            self._min_val_label.setText('-inf')
            self.func = lambda x: math.atan(x)
            self.ifunc = lambda x: math.tan(x)

        if config['max'] != float('inf'):
            self.max = float(config['max'])
            self._max_val_label.setText(str(self.max))
            self.func = lambda x: x
            self.ifunc = self.func
        else:
            self.max = 1e10000
            self._max_val_label.setText('inf')
            self.func = lambda x: math.atan(x)
            self.ifunc = lambda x: math.tan(x)

        self.scale = (self.func(self.max) - self.func(self.min)) / 100
        if self.scale == 0:
            self.setDisabled(True)

        self.offset = self.func(self.min)

        self._slider_horizontal.setRange(self.slider_value(self.min),
                                         self.slider_value(self.max))
        self._slider_horizontal.sliderReleased.connect(self.slider_released)
        self._slider_horizontal.sliderMoved.connect(self.update_text)

        self._paramval_lineEdit.setValidator(QDoubleValidator(
                                                    self.min, self.max,
                                                    4, self))
        self._paramval_lineEdit.editingFinished.connect(self.editing_finished)

        self._paramval_lineEdit.setText(str(config['default']))
        self._slider_horizontal.setSliderPosition(
                                     self.slider_value(config['default']))

    def get_value(self):
        return self.ifunc(self._slider_horizontal.value() * self.scale)

    def slider_value(self, value):
        return int(round((self.func(value)) / self.scale)) if self.scale else 0

    def slider_released(self):
        self.update_text(self.get_value())
        self._update(self.get_value())

    def update_text(self, value):
        self._paramval_lineEdit.setText(str(self.get_value()))
        rospy.logdebug(' DblEditor.update_text val=%s', str(value))

    def editing_finished(self):
        self._slider_horizontal.setSliderPosition(
                      self.slider_value(float(self._paramval_lineEdit.text())))
        self._update(float(self._paramval_lineEdit.text()))

    def update_value(self, val):
        self._slider_horizontal.setSliderPosition(
                                  self.slider_value(float(val)))
        self._paramval_lineEdit.setText(str(val))


class EnumEditor(EditorWidget):
    def __init__(self, updater, config):
        super(EnumEditor, self).__init__(updater, config)

        loadUi(ui_enum, self)

        try:
            enum = eval(config['edit_method'])['enum']
        except:
            rospy.logerr("reconfig EnumEditor) Malformed enum")
            return

        self.names = [item['name'] for item in enum]
        self.values = [item['value'] for item in enum]

        items = ["%s (%s)" % (self.names[i], self.values[i])
                 for i in range(0, len(self.names))]

        self._combobox.addItems(items)
        self._combobox.currentIndexChanged['int'].connect(self.selected)

    def selected(self, index):
        self._combobox.setCurrentIndex(self.values[index])
        self._update(self.values[index])

    #def update_value(self, val):
    #    self._combobox.setCurrentIndex(self.values.index(val))
