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

from python_qt_binding.QtCore import QDateTime, QObject


class Message(QObject):
    """
    Basic Message object. To directly access members use the get_data() function.
    """
    def __init__(self, msg=None):
        """
        :param msg: a log message to initialize the message object, ''Log''
        """
        super(Message, self).__init__()
        self._messagemembers = self.get_message_members()
        self._severity = {1: self.tr('Debug'), 2: self.tr('Info'), 4: self.tr('Warn'), 8: self.tr('Error'), 16: self.tr('Fatal')}
        self._time_format = None
        self.__color = 'black'
        if msg is not None:
            self._message = msg.msg
            self._severity = self._severity[msg.level]
            self._node = msg.name
            self._time = self.datestamp_to_qdatetime(msg.header.stamp.secs, msg.header.stamp.nsecs)
            self._topics = ', '.join(msg.topics)
            self._location = msg.file + ':' + msg.function + ':' + str(msg.line)

    def set_color(self, color):
        """
        :param color: Color Keyword ''str''
        """
        self.__color = color

    def get_color(self):
        """
        :returns: Color Keyword ''str''
        """
        return self.__color

    def _get_time(self):
        return self.__time

    def _set_time(self, time):
        """
        :param time: date and time to set ''QDateTime''
        """
        self.__time = time
        if self._time_format:
            self._time_string = self._time.toString(self._time_format)

    _time = property(_get_time, _set_time)

    @staticmethod
    def get_message_members():
        return ('_message', '_severity', '_node', '_time', '_topics', '_location')

    @staticmethod
    def header_print():
        members = Message.get_message_members()
        text = members[2][1:].capitalize() + ';'
        text += members[3][1:].capitalize() + ';'
        text += members[1][1:].capitalize() + ';'
        text += members[4][1:].capitalize() + ';'
        text += members[5][1:].capitalize() + ';'
        text += members[0][1:].capitalize() + '\n'
        return text

    def count(self):
        return len(self._messagemembers)

    def set_time_format(self, format):
        """
        :param format: formatting characters are defined in the QDateTime documentation ''str''
        """
        self._time_format = format
        self._time_string = self._time.toString(self._time_format)

    def time_as_string(self):
        """
        :returns: time in the format provided ''str''
        """
        return self._time.toString(self._time_format)

    def time_as_datestamp(self):
        """
        :returns: seconds with decimal fractions of a second, ''str''
        """
        seconds = self._time.toTime_t()
        seconds_in_qdate = QDateTime()
        seconds_in_qdate.setTime_t(seconds)
        msecs = seconds_in_qdate.msecsTo(self._time)
        return str(seconds) + '.' + str(msecs)

    def datestamp_to_qdatetime(self, secs, nsecs):
        """
        :param secs: seconds from a datestamp ''int''
        :param nsecs: nanoseconds from a datestamp ''int''
        :returns: converted time ''QDateTime''
        """
        temp_time = QDateTime()
        temp_time.setTime_t(int(secs))
        return temp_time.addMSecs(int(str(nsecs).zfill(9)[:3]))

    def load_from_array(self, rowdata):
        """
        :param rowdata:
            [0] = message, ''str''
            [1] = severity, ''str''
            [2] = node name, ''str''
            [3] = time in seconds including decimal, ''str''
            [4] = topic name, ''str''
            [5] = location value, ''str''
        """
        self._message = rowdata[0]
        self._severity = rowdata[1]
        self._node = rowdata[2]
        self._time = rowdata[3]
        self._topics = rowdata[4]
        self._location = rowdata[5]
        return self

    def file_load(self, text):
        """
        :param text: delmited message text as follows, node;time;severity;topics;location;"message" , ''str''
        """
        text = text[1:]
        sc_index = text.find('";"')
        if sc_index == -1:
            raise ValueError('File format is incorrect, missing ";" marker')
        self._node = text[:sc_index]
        text = text[text.find('";"') + 3:]
        sc_index = text.find('";"')
        if sc_index == -1:
            raise ValueError('File format is incorrect, missing ";" marker')
        sec, nsec = text[:sc_index].split('.')
        self._time = self.datestamp_to_qdatetime(sec, nsec)
        text = text[text.find('";"') + 3:]
        sc_index = text.find('";"')
        if sc_index == -1:
            raise ValueError('File format is incorrect, missing ";" marker')
        self._severity = text[:sc_index]
        text = text[text.find('";"') + 3:]
        sc_index = text.find('";"')
        if sc_index == -1:
            raise ValueError('File format is incorrect, missing ";" marker')
        self._topics = text[:sc_index]
        text = text[text.find('";"') + 3:]
        sc_index = text.find('";"')
        if sc_index == -1:
            raise ValueError('File format is incorrect, missing ";" marker')
        self._location = text[:sc_index]
        text = text[sc_index + 2:]
        text = text.replace('\\"', '"')
        self._message = text[1:-2]
        return

    def file_print(self):
        text = '"' + self._node + '";'
        text += '"' + self.time_as_datestamp() + '";'
        text += '"' + self._severity + '";'
        text += '"' + self._topics + '";'
        text += '"' + self._location + '";'
        altered_message = self._message.replace('"', '\\"')
        text += '"' + altered_message + '"\n'
        return text

    def pretty_print(self):
        text = self.tr('Node: ') + self._node + '\n'
        text += self.tr('Time: ') + self.time_as_datestamp() + '\n'
        text += self.tr('Severity: ') + self._severity + '\n'
        text += self.tr('Published Topics: ') + self._topics + '\n'
        text += '\n' + self._message + '\n'
        text += '\n' + 'Location:'
        text += '\n' + self._location + '\n\n'
        text += '-' * 100 + '\n\n'

        return text

    def get_data(self, col):
        return getattr(self, Message.get_message_members()[col])
