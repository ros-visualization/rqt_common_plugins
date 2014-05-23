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

# Design notes:
#
# The original rxbag plot widget displays the plot
#
# It has a settings menu which allows the user to add subplots
#  and assign arbitrary fields to subplots. possibly many fields in a single
#  subplot, or one field per subplot, or any othe rcombination
#  in particular, it is possible to add the same field to multiple subplots
# It doesn't appear able to add fields from different topics to the same plot
#  Since rqt_plot can do this, and it's useful for our application, it's worth
#  thinking about. If it makes the UI too cluttered, it may not be worth it
#  If it isn't possible due to the architecture of rqt_bag, it may not be
#  worth it
#
# Thoughts on new design:
#  a plottable field is anything which is either numeric, or contains
#  at least one plottable subfield (this is useful for enabling/disabling all
#  of the fields of a complex type)
#
#  for simple messages, we could display a tree view of the message fields
#  on top of the plot, along with the color for each plottable field. This gets
#  unweildy for large messages, because they'll use up too much screen space
#  displaying the topic list
#
#  It may be better to display the topic list for a plot as a dockable widget,
#  and be able to dismiss it when it isn't actively in use, similar to the
#  existing rxbag plot config window
#
#  The plot should be dockable and viewable. If it's a separate window, it
#  doesn't make sense to make it align with the timeline. This could be done
#  if someone wanted to implement a separate timeline view

from rqt_bag import MessageView

from python_qt_binding.QtGui import QWidget

class PlotView(MessageView):
    """
    Popup plot viewer
    """
    name = 'Plot'

    def __init__(self, timeline, parent):
        super(PlotView, self).__init__(timeline, parent)

        self._plot_widget = QWidget(parent) # use rqt plot widget... ?
        parent.layout().addWidget(self._plot_widget)

    def message_viewed(self, bag, msg_details):
        """
        refreshes the plot
        """
        TopicMessageView.message_viewed(self, bag, msg_details)
        topic, msg, t = msg_details[:3]
        if not msg:
            #self.set_plot(None, topic, 'no message')
            pass
        else:
            #self.set_plot(msg, topic, msg.header.stamp)
            pass

    def message_cleared(self):
        TopicMessageView.message_cleared(self)

