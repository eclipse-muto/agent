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

import rclpy
from rclpy.node import Node

from muto_msgs.msg import (
    MutoAction,
    MutoActionMeta,
    CommandInput,
    CommandOutput,
    PluginResponse,
)
from muto_msgs.srv import CommandPlugin

from agent.ros.node_commands import NodeCommands
from agent.ros.topic_commands import TopicCommands
from agent.ros.param_commands import ParamCommands

import json


PLUGINS = {"CommandPlugin": CommandPlugin}


class Command:
    """
    Class for executing commands.

    This class provides functionality for each commmands thats defined in the
    config file. When a command is executed, asynchronous call is made to the
    corresponding service and result is published to the muto_agent.

    Args:
        node: RosCommnadsPlugin class which is a subclass of Node.
        service: The name of the service to call.
        plugin: The plugin to use for the command.
    """

    def __init__(self, node, service, plugin):
        self.node = node
        self.service = service
        self.plugin = PLUGINS[plugin]

        self.client = self.node.create_client(self.plugin, self.service)
        self.req = CommandPlugin.Request()

    def execute(self, method, payload, meta):
        """
        Execute a command.

        When this method run, asynchronous call will be made to the service.

        Args:
            method: The command to execute (only needed for the CommandInput).
            payload: Payload of the command.
            meta: MutoActionMeta object which holds information about response
                topic.

        Returns:
            None if service is not found.
        """
        command_input = CommandInput()
        command_input.command = method
        command_input.payload = payload

        self.req.input = command_input
        self.payload = payload
        self.meta = meta

        if self.client.service_is_ready():
            self.future = self.client.call_async(self.req)
            self.future.add_done_callback(self.service_callback)

        else:
            self.node.get_logger().error("Service not ready.")
            return None

    def service_callback(self, future):
        """
        Callback function for the future object.

        This callback function is executed when the service call is completed.
        It retrieves the result from the Future object and publishes the
        result to the agent.

        Args:
            future: The Future object representing the service call.
        """
        try:
            result = future.result()
        except Exception as e:
            self.node.get_logger().error(f"Service call failed: {e}")

        self.node.publish_executed_command_result(result, self.payload, self.meta)


class ROSCommandsPlugin(Node):
    """
    A ROS node for handling commands which also communication with the agent.

    This class represents a ROS node that handles commands received from an
    agent. It provides functionality for executing commands, publishing command
    results, and constructing command output messages. The node subscribes to
    an agent topic to receive command requests and publishes command results to
    the agent.

    Attributes:
        agent_to_commands_topic (str): The name of the agent to commands topic.
        commands_to_agent_topic (str): The name of the commands to agent topic.
        commands (dict): A dictionary containing the loaded commands.
        pub_agent: Publisher for sending command execution results to agent.
        sub_agent: Subscriber for receiving command requests from an agent.
    """

    def __init__(self):
        super().__init__(
            node_name="commands_plugin",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        # Declare Parameters
        try:
            self.declare_parameter("agent_to_commands_topic", "msg_3")
            self.declare_parameter("commands_to_agent_topic", "msg4")
        except:
            pass

        # Initialize Parameters
        self.agent_to_commands_topic = self.get_parameter(
            "agent_to_commands_topic"
        ).value
        self.commands_to_agent_topic = self.get_parameter(
            "commands_to_agent_topic"
        ).value

        self.commands = self.load_commands()

        # ROS Related
        self.pub_agent = self.create_publisher(
            MutoAction, self.commands_to_agent_topic, 10
        )
        self.sub_agent = self.create_subscription(
            MutoAction, self.agent_to_commands_topic, self.agent_msg_callback, 10
        )

        # Command Service Definitions
        NodeCommands(self)
        TopicCommands(self)
        ParamCommands(self)

    def load_commands(self):
        """
        Load commands from the parameters file and return a dictionary
        containing the commands.

        Returns:
            A dictionary containing the loaded commands, or None if no commands
            are found.
        """
        # Get commands from config file. Return None if no commmands are found.
        commands = self.get_parameters_by_prefix("commands")

        if commands == {}:
            return None

        # Create dictionary of commands
        commands_dict = {}
        for i in commands:
            i = i.split(".")
            command_num = i[0]
            command_key = i[1]
            command_value = commands[f"{command_num}.{command_key}"].value

            if command_num not in commands_dict:
                commands_dict[command_num] = {}
            commands_dict[command_num][command_key] = command_value

        # Create dictionary containing commands as Command object
        command_objects = {}
        for command in commands_dict:
            name = commands_dict[command]["name"]
            service = commands_dict[command]["service"]
            plugin = commands_dict[command]["plugin"]

            command_objects[name] = Command(self, service, plugin)

        return command_objects

    def agent_msg_callback(self, data):
        """
        Callback function for processing agent messages.

        This callback function is invoked when an agent message is received. It
        attempts to execute the corresponding command of the specified method.
        If the command is not found, or an exception occurs during command
        execution, appropriate error messages are logged.

        Args:
            data: The agent message data object.
        """
        context = data.context
        method = data.method
        payload = data.payload
        meta = data.meta

        try:
            command = self.commands[method]
            command.execute(method, payload, meta)
        except KeyError:
            self.get_logger().error(f"Command '{method}' not found.")
            self.publish_error(f"Command '{method}' not found.", meta)
        except Exception as e:
            self.get_logger().error(f"{e}")

    def publish_executed_command_result(self, result, payload, meta):
        """
        Publish the executed command result to the agent.

        This method publishes the executed command result to the agent by
        constructing a MutoAction message with the specified result payload and
        meta data. The message is then published using the agent publisher.

        Args:
            result: The result object containing the executed command result.
            meta: The MutoActionMeta message associated with the command.
        """
        payload = json.loads(payload)
        payload["value"] = result.output.payload
        payload["path"] = payload["path"].replace("/inbox", "/outbox")

        msg_action = MutoAction()
        msg_action.context = ""
        msg_action.method = ""
        msg_action.payload = json.dumps(payload)
        msg_action.meta = meta

        self.pub_agent.publish(msg_action)

    def publish_telemetry(self, telemetry_data, meta):
        """
        Publish telemetry to the agent.

        This method publishes telemetry to the agent by constructing a MutoAction
        message with the specified telemetry payload and meta data. The message
        is then published using the agent publisher.

        Args:
            telemetry_data: Telemetry data.
            meta: The MutoActionMeta message associated with the command.
        """
        msg_telemetry_data_meta = MutoActionMeta()
        msg_telemetry_data_meta.response_topic = meta["topic"]
        msg_telemetry_data_meta.correlation_data = meta["correlation"]

        msg_telemetry_data = MutoAction()
        msg_telemetry_data.context = ""
        msg_telemetry_data.method = ""
        msg_telemetry_data.payload = telemetry_data
        msg_telemetry_data.meta = msg_telemetry_data_meta

        self.pub_agent.publish(msg_telemetry_data)

    def publish_error(self, error_message, meta):
        """
        Publish error message to the agent.

        Args:
            error_message: Error message.
            meta: The MutoActionMeta message associated with the command.
        """
        msg_telemetry_data = MutoAction()
        msg_telemetry_data.context = ""
        msg_telemetry_data.method = ""
        msg_telemetry_data.payload = error_message
        msg_telemetry_data.meta = meta

        self.pub_agent.publish(msg_telemetry_data)

    def construct_command_output_message(
        self, data, result_code=0, err_msg="", err_desc=""
    ):
        """
        Constructs CommandOutput message.

        This method constructs a command output message with the specified data,
        result code, error message, and error description.

        Args:
            data: The payload to be included in the command output message.
            result_code: The result code indicating the status of the command
                execution. Defaults to 0 which indicates successful operation.
            err_msg: The error message associated with the command execution,
                if applicable.
            err_desc: The error description providing more details about the
                error, if applicable.

        Returns:
            The constructed command output message.
        """
        msg_plugin_response = PluginResponse()
        msg_plugin_response.result_code = result_code
        msg_plugin_response.error_message = err_msg
        msg_plugin_response.error_description = err_desc

        msg = CommandOutput()
        msg.payload = json.dumps(data)
        msg.result = msg_plugin_response
        return msg


def main():
    rclpy.init()
    rsp = ROSCommandsPlugin()
    rclpy.spin(rsp)


if __name__ == "__main__":
    main()
