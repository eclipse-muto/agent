import json
import rospy
import std_msgs.msg as std_msgs
import muto_msgs.srv as muto_srv
import muto_msgs.msg as muto_msgs


PLUGINS = {
    "CommandPlugin": muto_srv.CommandPlugin
}


def to_commandoutput(data):
    response = muto_msgs.CommandOutput()
    response.payload = data
    response.result = muto_msgs.PluginResponse(resultCode=0, errorMessage="", errorDescription="")
    return response


def to_stdmsgs_string(message):
    msg = std_msgs.String()
    msg.data = message
    return msg
    
class Command(object):

    def __init__(self, command):
        # Initialize from spec
        self.command = command

    def execute(self, msg, payload):
        action = msg.method
        print("Execute pipeline for command:", action)
        service = self.command.get('service')
        plugin = self.command.get('plugin')
        if not service is None and not plugin is None:
            try:
                call = rospy.ServiceProxy(
                    service, PLUGINS[plugin])
                input = muto_msgs.CommandInput()
                input.command = action
                input.payload = json.dumps(payload)
                callResponse = call(input)
                return callResponse

            except rospy.ServiceException as e:
                print("Service call failed {}:{}".format(service, e))

