#
#  Copyright (c) 2023 Composiv.ai
#
# All rights reserved. This program and the accompanying materials
# are made available under the terms of the Eclipse Public License v2.0
# and Eclipse Distribution License v1.0 which accompany this distribution.
#
# Licensed under the  Eclipse Public License v2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# The Eclipse Public License is available at
#    http://www.eclipse.org/legal/epl-v20.html
# and the Eclipse Distribution License is available at
#   http://www.eclipse.org/org/documents/edl-v10.php.
#
# Contributors:
#    Composiv.ai - initial API and implementation
#
#
from muto_msgs.srv import CommandPlugin

import json


class ParamCommands():
    """
    Class for managing rosparam commands.

    This class provides functionality to handle rosparam commands such as
    listing parameters and retrieving parameter value. Also services are
    defined here.

    Args:
        node: RosCommnadsPlugin class which is a subclass of Node.

    """

    def __init__(self, node):
        self.node = node

        self.rosparam_list = self.node.create_service(CommandPlugin, "rosparam_list", self.callback_rosparam_list)
        self.rosparam_get = self.node.create_service(CommandPlugin, "rosparam_get", self.callback_rosparam_get)

    def callback_rosparam_list(self, request, response):
        """
        Callback function for handling rosparam_list service request.

        Retrieves a list of parameters, get the value for each parameter
        constructs a JSON-formatted result, and sends it as the
        output of the response.

        Args:
            request: The request message object.
            response: The response message object.

        Returns:
            The modified response message object.
        """
        result = {"params": []}

        parameters = self.node.get_parameters_by_prefix("")

        for parameter in parameters:
            if not parameter.startswith("commands"):
                value = self.node.get_parameter(parameter).value
                result["params"].append({"name": parameter, "value": value})

        response.output = self.node.construct_command_output_message(result)

        return response

    def callback_rosparam_get(self, request, response):
        """
        Callback function for handling rosparam_get service request.

        Retrieves the value of a selected parameter, constructs a JSON-formatted
        result, and sends it as the output of the response.

        Args:
            request: The request message object.
            response: The response message object.

        Returns:
            The modified response message object.
        """
        payload = json.loads(request.input.payload)
        requested_param = payload["param"]

        result = {"status": "NOTFOUND"}

        try:
            value = self.node.get_parameter(requested_param).value
            result = {"name": requested_param, "value": value}
        except:
            self.node.get_logger().error(f"{requested_param} not found.")

        response.output = self.node.construct_command_output_message(result)

        return response