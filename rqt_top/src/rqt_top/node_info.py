# Copyright (c) 2013, Oregon State University
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Oregon State University nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL OREGON STATE UNIVERSITY BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Author Dan Lazewatsky/lazewatd@engr.orst.edu

import rosnode
import rospy
import xmlrpclib
import psutil

ID = '/NODEINFO'

class NodeInfo(object):
    nodes = dict()
    def get_node_info(self, node_name):
        node_api = rosnode.get_api_uri(rospy.get_master(), node_name)
        code, msg, pid = xmlrpclib.ServerProxy(node_api[2]).getPid(ID)
        if node_name in self.nodes:
            return self.nodes[node_name]
        else:
            try:
                p = psutil.Process(pid)
                self.nodes[node_name] = p
                return p
            except:
                return False


    def get_all_node_info(self):
        infos = []
        for node_name in rosnode.get_node_names():
            info = self.get_node_info(node_name)
            if info is not False: infos.append((node_name, info))
        return infos

    def get_all_node_fields(self, fields):
        # import pdb; pdb.set_trace()
        processes = self.get_all_node_info()
        infos = []
        for name, p in processes:
            infos.append(p.as_dict(fields + ['cmdline', 'get_memory_info']))
            infos[-1]['node_name'] = name
        return infos

    def kill_node(self, node_name):
        success, fail = rosnode.kill_nodes([node_name])
        return node_name in success
