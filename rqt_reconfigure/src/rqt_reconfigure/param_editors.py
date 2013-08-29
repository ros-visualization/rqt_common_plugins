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
    '''
    This class is abstract -- its child classes should be instantiated.

    There exist two kinds of "update" methods:
    - _update_paramserver for Parameter Server.
    - update_value for the value displayed on GUI.
    '''

    def __init__(self, updater, config):
        '''
        @param updater: A class that extends threading.Thread.
        @type updater: rqt_reconfigure.param_updater.ParamUpdater
        '''

        super(EditorWidget, self).__init__()

        self._updater = updater
        self.param_name = config['name']

        self.old_value = None

    def _update_paramserver(self, value):
        '''
        Update the value on Parameter Server.
        '''
        if value != self.old_value:
            self.update_configuration(value)
            self.old_value = value

    def update_value(self, value):
        '''
        To be overridden in subclass.

        Update the value that's displayed on the arbitrary GUI component
        based on user's input.
        '''
        pass

    def update_configuration(self, value):
        self._updater.update({self.param_name: value})

    def display(self, grid):
        '''
        Should be overridden in subclass.

        :type grid: QFormLayout
        '''
        self._paramname_label.setText(self.param_name)
#        label_paramname = QLabel(self.param_name)
#        label_paramname.setWordWrap(True)
        self._paramname_label.setMinimumWidth(100)
        grid.addRow(self._paramname_label, self)

    def close(self):
        '''
        Should be overridden in subclass.
        '''
        pass


class BooleanEditor(EditorWidget):
    def __init__(self, updater, config):
        super(BooleanEditor, self).__init__(updater, config)
        loadUi(ui_bool, self)

        self.update_value(config['default'])
        self._checkbox.clicked.connect(self._update_paramserver)
        #TODO: Maybe add slot for stateChanged; it can be just:
        #      self._checkbox.stateChanged.connect(self._update_paramserver)

    def update_value(self, value):
        self._checkbox.setChecked(value)


class StringEditor(EditorWidget):
    def __init__(self, updater, config):
        super(StringEditor, self).__init__(updater, config)
        loadUi(ui_str, self)

        # Emit signal when cursor leaves the text field.
        self._paramval_lineedit.editingFinished.connect(self.edit_finished)
        # Add textChanged to capture the change while cursor is still in
        # the text field.
        self._paramval_lineedit.textChanged.connect(self.edit_finished)

    def update_value(self, value):
        rospy.logdebug('StringEditor update_value={}'.format(value))
        self._paramval_lineedit.setText(value)

    def edit_finished(self):
        rospy.logdebug('StringEditor edit_finished val={}'.format(
                                              self._paramval_lineedit.text()))
        self._update_paramserver(self._paramval_lineedit.text())


class IntegerEditor(EditorWidget):
    def __init__(self, updater, config):
        super(IntegerEditor, self).__init__(updater, config)

        loadUi(ui_int, self)

        self._min = int(config['min'])
        self._max = int(config['max'])
        self._min_val_label.setText(str(self._min))
        self._max_val_label.setText(str(self._max))

        self._slider_horizontal.setRange(self._min, self._max)

        # TODO: Fix that the naming of _paramval_lineEdit instance is not
        #       consistent among Editor's subclasses.
        self._paramval_lineEdit.setValidator(QIntValidator(self._min,
                                                           self._max, self))

        # Connect slots
        self._paramval_lineEdit.textChanged.connect(self._text_edited)
        self._slider_horizontal.sliderReleased.connect(self._slider_released)
        #self._slider_horizontal.sliderMoved.connect(self._update_text_gui)
        # valueChanged gets called when groove is clicked on.
        #self._slider_horizontal.valueChanged.connect(self.update_value)

        # TODO: This should not always get set to the default it should be the
        # current value
        self._paramval_lineEdit.setText(str(config['default']))
        self._slider_horizontal.setSliderPosition(int(config['default']))

    def _text_edited(self):
        self._update_paramserver(int(self._paramval_lineEdit.text()))

    def _slider_released(self):
        '''Slot for mouse being released from slider.'''
        _slider_val = self._slider_horizontal.value()
        self._update_text_gui(_slider_val)

    def _update_text_gui(self, value):
        rospy.logdebug(' IntegerEditor._update_text_gui val=%s', str(value))
        self._paramval_lineEdit.setText(str(value))
        # Run self._update_paramserver to update the value on PServer
        self._update_paramserver(value)
        self.update_value(value)

    def update_value(self, val):
        # Can be redundant when the trigger is the move of slider.
        self._slider_horizontal.setSliderPosition(val)

        self._paramval_lineEdit.setText(str(val))


class DoubleEditor(EditorWidget):
    def __init__(self, updater, config):
        super(DoubleEditor, self).__init__(updater, config)

        loadUi(ui_num, self)

        # Handle unbounded doubles nicely
        if config['min'] != -float('inf'):
            self._min = float(config['min'])
            self._min_val_label.setText(str(self._min))
            self._func = lambda x: x
            self._ifunc = self._func
        else:
            self._min = -1e10000
            self._min_val_label.setText('-inf')
            self._func = lambda x: math.atan(x)
            self._ifunc = lambda x: math.tan(x)

        if config['max'] != float('inf'):
            self._max = float(config['max'])
            self._max_val_label.setText(str(self._max))
            self._func = lambda x: x
            self._ifunc = self._func
        else:
            self._max = 1e10000
            self._max_val_label.setText('inf')
            self._func = lambda x: math.atan(x)
            self._ifunc = lambda x: math.tan(x)

        self.scale = (self._func(self._max) - self._func(self._min)) / 100
        if self.scale == 0:
            self.setDisabled(True)

        self.offset = self._func(self._min)

        self._slider_horizontal.setRange(self._get_value_slider(self._min),
                                         self._get_value_slider(self._max))

        self._paramval_lineEdit.setValidator(QDoubleValidator(
                                                    self._min, self._max,
                                                    8, self))
        self._paramval_lineEdit.setText(str(config['default']))
        self._slider_horizontal.setSliderPosition(
                                     self._get_value_slider(config['default']))

        # Connect slots
        self._paramval_lineEdit.textChanged.connect(self._text_edited)
        self._slider_horizontal.sliderReleased.connect(self._slider_released)

    def _text_edited(self):
        '''Slot for text changed event in text field.'''
        self._update_paramserver(float(self._paramval_lineEdit.text()))

    def _get_value_textfield(self):
        '''@return: Current value in text field.'''
        return self._ifunc(self._slider_horizontal.value() * self.scale)

    def _slider_released(self):
        '''Slot for mouse being released from slider.'''
        _slider_val = self._get_value_textfield()
        self._update_text_gui(_slider_val)

    def _get_value_slider(self, value):
        '''
        @rtype: double
        '''
        return int(round((self._func(value)) / self.scale)) if self.scale else 0

    def _update_text_gui(self, value):
        self._paramval_lineEdit.setText(str(value))
        rospy.loginfo(' DblEditor._update_text_gui val=%s', str(value))
        # Run self._update_paramserver to update the value on PServer
        self._update_paramserver(value)
        self.update_value(value)

    def update_value(self, val):
        # Can be redundant when the trigger is the move of slider.
        self._slider_horizontal.setSliderPosition(
                                            self._get_value_slider(float(val)))

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
        self._update_paramserver(self.values[index])

    def update_value(self, val):
        # apparently, when the currentIndexChanged signal is emitted, current 
        # index internally is still the old value, why the setCurrentIndex call
        # below triggers another selected() call ... couldn't fin a better 
        # solution than this. 
        self._combobox.currentIndexChanged['int'].disconnect()
        self._combobox.setCurrentIndex(self.values.index(val))
        self._combobox.currentIndexChanged['int'].connect(self.selected)
