# -----------------------------------------------------------------------------
# Software License Agreement (BSD License)
#
#  Copyright (c) 2014-2017, JSK Laboratory.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/o2r other materials provided
#     with the distribution.
#   * Neither the name of the JSK Laboratory nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
# -----------------------------------------------------------------------------

import argparse

from rqt_gui_py.plugin import Plugin

from .widget import PlotHistogramWidget


class PlotHistogram(Plugin):
    def __init__(self, context):
        super(PlotHistogram, self).__init__(context)
        self.setObjectName('PlotHistogram')
        self._args = self._parse_args(context.argv())
        self._widget = PlotHistogramWidget(self._args.topic)
        context.add_widget(self._widget)

    def _parse_args(self, argv):
        parser = argparse.ArgumentParser(prog='rqt_plot_hist', add_help=True)
        PlotHistogram.add_arguments(parser)
        args = parser.parse_args(argv)
        return args

    @staticmethod
    def add_arguments(parser):
        group = parser.add_argument_group('Options for rqt_plot_hist plugin')
        group.add_argument('topic', nargs='?', help='Topic to plot')
