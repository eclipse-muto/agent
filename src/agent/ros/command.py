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
from agent.ros.async_serviceproxy import AsyncServiceProxy, AsyncServiceProxyException
import rospy
import std_msgs.msg as std_msgs
import muto_msgs.srv as muto_srv
import muto_msgs.msg as muto_msgs
import paho.mqtt.client as mqtt
import paho.mqtt.packettypes as packettypes

PLUGINS = {
    "CommandPlugin": muto_srv.CommandPlugin
}


def to_commandoutput(data):
    response = muto_msgs.CommandOutput()
    response.payload = data
    response.result = muto_msgs.PluginResponse(resultCode=0, errorMessage="", errorDescription="")
    return response

def to_responsetopic(mqttclient, req, result):
    if hasattr(req,'input'):
        input = json.loads(req.input.payload)
        if input['target'] is not None:
            response = json.dumps(result)
            properties=mqtt.Properties(packettypes.PacketTypes.PUBLISH)
            properties.CorrelationData=(input['target']['correlation']).encode('utf-8')
            mqttclient.publish(input['target']['topic'],response,properties=properties)

def to_stdmsgs_string(message):
    msg = std_msgs.String()
    msg.data = message
    return msg
    
class Command(object):

    def __init__(self, command):
        # Initialize from spec
        self.command = command
        self.srv_client = None
        self.message_type = {}
        service = self.command.get('service')
        self.offline_services = [service]

    def register_plugin(self):
        service = self.command.get('service')
        plugin = self.command.get('plugin')
        try:
            self.srv_client = AsyncServiceProxy(service,PLUGINS[plugin])
            if service in self.offline_services:
                self.offline_services.remove(service)
        except AsyncServiceProxyException:
            if service not in self.offline_services:
                self.offline_services.append(service)


    def execute(self, msg, payload):
        action = msg.method
        print("Execute plugin for command:", action)
        service = self.command.get('service')
        plugin = self.command.get('plugin')
        if not service is None and not plugin is None:
            try:
                if service in self.offline_services:
                    rospy.logerr("command {} was not played because the service server was unavailable. Trying to reconnect..."
                                .format(service))
                    self.register_plugin()

                input = muto_msgs.CommandInput()
                input.command = action
                input.payload = json.dumps(payload)
                callResponse = self.srv_client(input)
                return callResponse

            except rospy.ServiceException as e:
                print("Service call failed {}:{}".format(service, e))

