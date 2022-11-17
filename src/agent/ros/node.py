#
#  Copyright (c) 2022 Composiv.ai, Eteration A.S. and others
#
# All rights reserved. This program and the accompanying materials
# are made available under the terms of the Eclipse Public License v2.0
# and Eclipse Distribution License v1.0 which accompany this distribution.
#
# The Eclipse Public License is available at
#    http://www.eclipse.org/legal/epl-v10.html
# and the Eclipse Distribution License is available at
#   http://www.eclipse.org/org/documents/edl-v10.php.
#
# Contributors:
#    Composiv.ai, Eteration A.S. - initial API and implementation
#
#

import json
import socket
import rospy
import rosgraph
import rosnode
from muto_msgs.srv import *
from muto_msgs.msg import *
import agent.ros.command as command
import muto_msgs.srv as muto_srv


class RosNodeCommands(object):

    def __init__(self, mqtt_client):
        self.mqtt_client = mqtt_client

        self.rosnode_list = rospy.Service(
            "rosnode_list", muto_srv.CommandPlugin, self.handle_node_list)
        self.rosnode_info = rospy.Service(
            "rosnode_info", muto_srv.CommandPlugin, self.handle_node_info)
        self.rosnode_kill = rospy.Service(
            "rosnode_kill", muto_srv.CommandPlugin, self.handle_node_kill)
        self.rosnode_ping = rospy.Service(
            "rosnode_ping", muto_srv.CommandPlugin, self.handle_node_ping)        


    def handle_node_list(self, req):  # list active node
        nodeNames = rosnode.get_node_names()
        result = { "nodes": []}
        for name in nodeNames:
            info = self.get_node_info(name)
            result["nodes"].append({ "name": name, "info": info})
        msg = command.to_stdmsgs_string(json.dumps(result))
        command.to_responsetopic(self.mqtt_client, req, result)
        return command.to_commandoutput(msg.data)

    def handle_node_info(self, req):  # print information about node
        payload = json.loads(req.input.payload)
        requestedNode = payload["node"]
        if not requestedNode is None:
            info = self.get_node_info(requestedNode)
            msg = command.to_stdmsgs_string(json.dumps(info))
            command.to_responsetopic(self.mqtt_client, req, info)
            return command.to_commandoutput(msg.data)
        return command.to_commandoutput("{}")

    def handle_node_kill(self, req):  # kill a running node
        payload = json.loads(req.input.payload)
        requestedNode = payload["node"]
        nodeNames = rosnode.get_node_names()
        status = { "status": "NOTFOUND"}
        if requestedNode in nodeNames:
            rosnode.kill_nodes(requestedNode)
            status = { "status": "KILLED", "node": requestedNode}
        msg = command.to_stdmsgs_string(json.dumps(status))
        return command.to_commandoutput(msg.data)

    def handle_node_ping(self, req):
        payload = json.loads(req.input.payload)
        requestedNode = payload["node"]
        nodeNames = rosnode.get_node_names()
        status = { "status": "NOTFOUND"}
        if requestedNode in nodeNames:
            node_ping = rosnode.rosnode_ping(requestedNode, max_count=1)
            status = { "status": node_ping, "node": requestedNode}
            command.to_responsetopic(self.mqtt_client, req, status)

        msg = command.to_stdmsgs_string(json.dumps(status))
        return command.to_commandoutput(msg.data)

    def get_node_info(self, node_name):
        def topic_type(t, pub_topics):
            matches = [t_type for t_name, t_type in pub_topics if t_name == t]
            if matches:
                return matches[0]
            return 'unknown type'

        master = rosgraph.Master('/rosnode')

        # go through the master system state first
        try:
            state = master.getSystemState()
            pub_topics = master.getPublishedTopics('/')
        except socket.error:
            raise rosnode.ROSNodeIOException("Unable to communicate with master!")
        pubs = sorted([t for t, l in state[0] if node_name in l])
        subs = sorted([t for t, l in state[1] if node_name in l])
        srvs = sorted([t for t, l in state[2] if node_name in l])

        info = {"name": node_name, "pubs": [], "subs": [], "services": []}
        if pubs:
            for top in pubs:
                info["pubs"].append({"topic": top, "type": topic_type(top, pub_topics)})
        if subs:
            for top in subs:
                info["subs"].append({"topic": top, "type": topic_type(top, pub_topics)})
        if srvs:
            for srv in srvs:
                info["services"].append(srv)
        return info
