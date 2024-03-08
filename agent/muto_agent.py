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

from std_msgs.msg import String
from muto_msgs.msg import Gateway, MutoAction

import re


class MutoAgent(Node):
    """TODO add documentation."""

    def __init__(self):
        super().__init__("muto_agent")

        # Declare Parameters
        self.declare_parameter("stack_topic", "stack")
        self.declare_parameter("twin_topic", "twin")

        self.declare_parameter("agent_to_gateway_topic", "agent_to_gateway")
        self.declare_parameter("gateway_to_agent_topic", "gateway_to_agent")

        self.declare_parameter("agent_to_commands_topic", "agent_to_command")
        self.declare_parameter("commands_to_agent_topic", "command_to_agent")

        # Initialize Parameters
        self.stack_topic = self.get_parameter("stack_topic").value
        self.twin_topic = self.get_parameter("twin_topic").value
        self.agent_to_gateway_topic = self.get_parameter("agent_to_gateway_topic").value
        self.gateway_to_agent_topic = self.get_parameter("gateway_to_agent_topic").value

        self.agent_to_commands_topic = self.get_parameter(
            "agent_to_commands_topic"
        ).value
        self.commands_to_agent_topic = self.get_parameter(
            "commands_to_agent_topic"
        ).value

        # ROS Related
        self.sub_gateway = self.create_subscription(
            Gateway, self.gateway_to_agent_topic, self.gateway_msg_callback, 10
        )
        self.pub_gateway = self.create_publisher(
            Gateway, self.agent_to_gateway_topic, 10
        )

        self.sub_stack = self.create_subscription(
            String, self.twin_topic, self.composer_msg_callback, 10
        )
        self.pub_stack = self.create_publisher(MutoAction, self.stack_topic, 10)

        self.sub_commands = self.create_subscription(
            MutoAction, self.commands_to_agent_topic, self.commands_msg_callback, 10
        )
        self.pub_commands = self.create_publisher(
            MutoAction, self.agent_to_commands_topic, 10
        )

    def gateway_msg_callback(self, data):
        """
        Callback function of gateway subscriber.

        Routes messages received from gateway to the respective module.

        Args:
            data: Gateway message.
        """
        # Parse Data
        topic = data.topic
        payload = data.payload
        meta = data.meta

        # Parse Topic
        type_, method = self.parse_topic(topic)

        # According to the request type, run the relevant method
        if type_ == "ping":
            self.respond_to_ping(payload, meta)
        elif type_ == "stack":
            self.send_to_composer(payload, meta, method)
        elif type_ == "agent":
            self.send_to_commands_plugin(payload, meta, method)

    def respond_to_ping(self, payload, meta):
        """
        Respond to vehicle ping message.

        Args:
            payload: Payload from the gateway message.
            meta: Meta from the gateway message.
        """
        msg = Gateway()
        msg.topic = ""
        msg.payload = payload.replace("/inbox", "/outbox")
        msg.meta = meta

        self.pub_gateway.publish(msg)

    def composer_msg_callback(self, data):
        """TODO add docs."""
        # TODO implement
        pass

    def commands_msg_callback(self, data):
        """
        Callback function of commands_plugin subscriber.

        Routes messages received from commands_plugin to ditto gateway.

        Args:
            data: MutoAction message.
        """
        msg = Gateway()
        msg.topic = ""
        msg.payload = data.payload
        msg.meta = data.meta

        self.pub_gateway.publish(msg)

    def parse_topic(self, topic):
        """
        Parse topic from gateway message.

        This method parses topic and detects type and method.

        Args:
            topic: Topic from gateway message.

        Returns:
            A tuple (type, method), where type is either stack or agent and
            method is command to run. Will return (None, None) tuple if topic
            doesn't involve "stack" or "agent" keywords.
        """
        try:
            if "telemetry" in topic:
                type_, method = None, None
            elif "ping" in topic:
                type_, method = "ping", None
            elif "stack" in topic:
                method = re.findall(".*/stack/commands/(.*)", topic)
                type_, method = "stack", method[0]
            elif "agent" in topic:
                method = re.findall(".*/agent/commands/(.*)", topic)
                type_, method = "agent", method[0]
            return type_, method
        except:
            self.get_logger().error(f"Invalid topic!")
            return None, None

    def send_to_composer(self, payload, meta, method):
        """
        Send MutoAction message to composer.

        Construct MutoActionMeta and MutoAction messages and send to
        composer.

        Args:
            payload: Payload from the gateway message.
            meta: Meta from the gateway message.
            method: Method extracted from gateway message topic.
        """

        msg_action = MutoAction()
        msg_action.context = ""
        msg_action.method = method
        msg_action.payload = payload
        msg_action.meta = meta

        self.pub_stack.publish(msg_action)

    def send_to_commands_plugin(self, payload, meta, method):
        """
        Send MutoAction message to commands plugin.

        Construct MutoActionMeta and MutoAction messages and send to
        commands plugin.

        Args:
            payload: Payload from the gateway message.
            meta: Meta from the gateway message.
            method: Method extracted from gateway message topic.
        """
        msg_action = MutoAction()
        msg_action.context = ""
        msg_action.method = method
        msg_action.payload = payload
        msg_action.meta = meta

        self.pub_commands.publish(msg_action)


def main():
    rclpy.init()
    agent = MutoAgent()
    rclpy.spin(agent)


if __name__ == "__main__":
    main()
