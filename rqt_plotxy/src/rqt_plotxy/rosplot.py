#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2016, Rafael Bailon-Ruiz, LAAS-CNRS
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
#  * Neither the name of LAAS-CNRS nor the names of its
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

import string
import sys
import threading
import time

import numpy as np

import rosgraph
import roslib.message
import roslib.names
import rospy

from rqt_plot import rosplot


class ROSDataXY(object):
    """
    Subscriber to two ROS topics that buffers incoming data.
    """

    def __init__(self, topics, start_time, precision_digits=3):
        self.ros_data_x = rosplot.ROSData(topics[0], start_time)
        self.ros_data_y = rosplot.ROSData(topics[1], start_time)

        self.precision = precision_digits

        self.prev1 = []
        self.prev2 = []

    def close(self):
        self.ros_data_x.close()
        self.ros_data_y.close()

    def next(self):
        datax = self.ros_data_x.next()
        datay = self.ros_data_y.next()

        data1 = self.prev1 + zip(np.around(datax[0], decimals=self.precision),
                                 datax[1])
        data2 = self.prev2 + zip(np.around(datay[0], decimals=self.precision),
                                 datay[1])

        result = [[], []]

        while data1 and data2:
            p1, p2 = data1[0], data2[0]
            if p1[0] == p2[0]:
                result[0].append(p1[1])
                result[1].append(p2[1])
                data1 = data1[1:]
                data2 = data2[1:]
            elif p1[0] < p2[0]:
                data1 = data1[1:]
            else:  # p1[0] > p2[0]
                data2 = data2[1:]
        else:
            self.prev1 = data1
            self.prev2 = data2

        return result

    def generate_field_evals(fields_x, fields_y):
        fx = self.ros_data_x.generate_field_evals(fields_x)
        fy = self.ros_data_x.generate_field_evals(fields_y)

        return fx, fy

    @property
    def error(self):
        return self.ros_data_y.error
