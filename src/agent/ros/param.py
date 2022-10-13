import json
import rospy
import rosparam

import agent.ros.command as command

import muto_msgs.srv as muto_srv




class RosParamCommands(object):

    def __init__(self, mqtt_client):
        self.mqtt_client = mqtt_client

        self.rosparam_get = rospy.Service(
            "rosparam_get", muto_srv.CommandPlugin, self.handle_param_get)
        self.rosparam_set = rospy.Service(
            "rosparam_set", muto_srv.CommandPlugin, self.handle_param_set)
        self.rosparam_list = rospy.Service(
            "rosparam_list", muto_srv.CommandPlugin, self.handle_param_list)
        self.rosparam_delete = rospy.Service(
            "rosparam_delete", muto_srv.CommandPlugin, self.handle_param_delete)
    


    def handle_param_get(self, req):
        payload = json.loads(req.input.payload)
        requested_key = payload["param"]
        if rospy.has_param(requested_key):
            msg = command.to_stdmsgs_string(str(rosparam.get_param(requested_key)))
            return command.to_commandoutput(msg.data)
        return command.to_commandoutput("{}")


    def handle_param_set(self, req):
        payload = json.loads(req.input.payload)
        requested_key = payload["param"]
        new_value = payload["value"]
        param = {"paramKey": requested_key,
                 "prevParamValue": "", "newParamValue": ""}
        if rospy.has_param(requested_key):
            # regex should be implemented and prev value - current val should be checked
            param["prevParamValue"] = rosparam.get_param(requested_key)
            rospy.set_param(requested_key, new_value)
            param["newParamValue"] = rosparam.get_param(requested_key)
            if param["newParamValue"] == new_value:
                msg = command.to_stdmsgs_string(str(param))
        else:
            msg = command.to_stdmsgs_string(
                " Make Sure That {} exists !".format(requested_key))
        return command.to_commandoutput(msg.data)

    def handle_param_list(self, req):
        parameterNames = rospy.get_param_names()
        if len(parameterNames) > 1:
            msg = command.to_stdmsgs_string(str(parameterNames))
        else:
            msg = command.to_stdmsgs_string("Something Went Wrong!")
        return command.to_commandoutput(msg.data)

    def handle_param_delete(self, req):  # Delete a parameter value
        payload = json.loads(req.input.payload)
        requested_key = payload["param"]
        if rospy.has_param(requested_key):
            if rosparam.get_param(requested_key) is not None:
                rosparam.delete_param(requested_key)
                msg = command.to_stdmsgs_string((" Parameter value of {} has been deleted !".format(requested_key)))

        else:
            msg = command.to_stdmsgs_string(" Make Sure That {} exists !".format(requested_key))
        return command.to_commandoutput(msg.data)


