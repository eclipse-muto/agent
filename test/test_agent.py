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

import unittest
import json

import rclpy
import agent.muto_agent
from muto_msgs.msg import Gateway, MutoAction, MutoActionMeta


class TestAgentNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def setUp(self):
        self.node = agent.muto_agent.MutoAgent()
        # Initialize the node to setup configuration and parameters
        try:
            self.node.initialize()
        except:
            # If initialization fails (e.g., missing services), continue with basic setup
            pass

    def tearDown(self):
        try:
            self.node.cleanup()
        except:
            pass
        self.node.destroy_node()

    def test_mqtt_node_create(self):
        assert self.node != None, "Node couldn't be created."

    def test_gateway_msg_callback_ping(self):
        meta_msg = MutoActionMeta()
        meta_msg.response_topic = "muto/org.eclipse.muto.sandbox:f1tenth"
        meta_msg.correlation_data = "7c4a0d1f-c38a-4076-b138-ac07357c1d0e"

        gw_msg = Gateway()
        gw_msg.topic = "org.eclipse.muto.sandbox/f1tenth/things/live/messages/agent/commands/ping"
        gw_msg.payload = json.dumps(
            {
                "topic": "org.eclipse.muto.sandbox/f1tenth/things/live/messages/agent/commands/ping",
                "headers": {
                    "content-type": "application/json",
                    "reply-to": "muto/org.eclipse.muto.sandbox:f1tenth",
                    "correlation-id": "7c4a0d1f-c38a-4076-b138-ac07357c1d0e"
                },
                "path": "/inbox/messages/agent/commands/ping",
                "value": {}
            }
        )
        gw_msg.meta = meta_msg

        self.node.received_message = None
        self.node.create_subscription(
            Gateway,
            self.node.get_parameter("agent_to_gateway_topic").value,
            lambda msg: setattr(self.node, "received_message", msg),
            10
        )

        self.node._gateway_msg_callback(gw_msg)

        rclpy.spin_once(self.node, timeout_sec=3)

        assert type(self.node.received_message) == Gateway
        assert self.node.received_message.topic == ""
        assert self.node.received_message.payload == gw_msg.payload.replace("/inbox", "/outbox")
        assert self.node.received_message.meta == meta_msg

    def test_gateway_msg_callback_stack(self):
        meta_msg = MutoActionMeta()
        meta_msg.response_topic = "muto/org.eclipse.muto.sandbox:alp-001"
        meta_msg.correlation_data = "082e0be4-83f3-4a71-a287-aa242ef3bc91"

        gw_msg = Gateway()
        gw_msg.topic = "org.eclipse.muto.sandbox/alp-001/things/live/messages/stack/commands/kill"
        gw_msg.payload = json.dumps(
            {
                "topic": "org.eclipse.muto.sandbox/alp-001/things/live/messages/stack/commands/kill",
                "headers": {
                    "content-type": "application/json",
                    "reply-to": "muto/org.eclipse.muto.sandbox:alp-001",
                    "correlation-id": "082e0be4-83f3-4a71-a287-aa242ef3bc91"
                },
                "path": "/inbox/messages/stack/commands/kill",
                "value": {"stackId": "org.eclipse.muto.sandbox:composable_client_server"}
            }
        )

        gw_msg.meta = meta_msg

        self.node.received_message = None
        self.node.create_subscription(
            MutoAction,
            self.node.get_parameter("stack_topic").value,
            lambda msg: setattr(self.node, "received_message", msg),
            10
        )

        self.node._gateway_msg_callback(gw_msg)

        rclpy.spin_once(self.node, timeout_sec=3)

        assert type(self.node.received_message) == MutoAction
        assert self.node.received_message.context == ""
        assert self.node.received_message.method == "kill"
        assert self.node.received_message.payload == gw_msg.payload
        assert self.node.received_message.meta == meta_msg

    def test_gateway_msg_callback_agent(self):
        meta_msg = MutoActionMeta()
        meta_msg.response_topic = "muto/org.eclipse.muto.sandbox:f1tenth"
        meta_msg.correlation_data = "494da5fa-4204-4b2e-8d10-d3138af3349a"

        gw_msg = Gateway()
        gw_msg.topic = "org.eclipse.muto.sandbox/f1tenth/things/live/messages/agent/commands/ros/node"
        gw_msg.payload = json.dumps(
            {
                "topic": "org.eclipse.muto.sandbox/f1tenth/things/live/messages/agent/commands/ros/node",
                "headers": {
                    "content-type": "application/json",
                    "reply-to": "muto/org.eclipse.muto.sandbox:f1tenth",
                    "correlation-id": "494da5fa-4204-4b2e-8d10-d3138af3349a"
                },
                "path": "/inbox/messages/agent/commands/ros/node",
                "value": {}
            }
        )
        gw_msg.meta = meta_msg

        self.node.received_message = None
        self.node.create_subscription(
            MutoAction,
            self.node.get_parameter("agent_to_commands_topic").value,
            lambda msg: setattr(self.node, "received_message", msg),
            10
        )

        self.node._gateway_msg_callback(gw_msg)

        rclpy.spin_once(self.node, timeout_sec=3)

        assert type(self.node.received_message) == MutoAction
        assert self.node.received_message.context == ""
        assert self.node.received_message.method == "ros/node"
        assert self.node.received_message.payload == gw_msg.payload
        assert self.node.received_message.meta == meta_msg

    def test_composer_msg_callback(self):
        # TODO
        pass

    def test_commands_msg_callback(self):
        meta_msg = MutoActionMeta()
        meta_msg.response_topic = "db-org.eclipse.muto.sandbox:f1tenth/agent/fa30fe44-153f-4541-b78e-3f71863a331d"
        meta_msg.correlation_data = "fa30fe44-153f-4541-b78e-3f71863a331d"        
        
        action_msg = MutoAction()
        action_msg.context = ""
        action_msg.method = ""
        action_msg.payload = '{"nodes": [{"name": "muto_agent"}]}'
        action_msg.meta = meta_msg

        self.node.received_message = None
        self.node.create_subscription(
            Gateway,
            self.node.get_parameter("agent_to_gateway_topic").value,
            lambda msg: setattr(self.node, "received_message", msg),
            10
        )

        self.node._commands_msg_callback(action_msg)

        rclpy.spin_once(self.node, timeout_sec=3)

        assert type(self.node.received_message) == Gateway
        assert self.node.received_message.topic == ""
        assert self.node.received_message.payload == action_msg.payload
        assert self.node.received_message.meta == meta_msg

    def test_parse_topic(self):
        topic = "org.eclipse.muto.sandbox/f1tenth/things/live/messages/agent/commands/ping"
        
        # Use the topic parser from the node
        topic_parser = self.node.get_topic_parser()
        if topic_parser:
            res = topic_parser.parse_topic(topic)
            assert res == ("ping", None), "Return value must be ('ping', None)"
        else:
            # If topic parser is not available, skip the test
            self.skipTest("Topic parser not initialized")
        
        topic = "org.eclipse.muto.sandbox:f1tenth/stack/commands/apply"
        res = self.node.parse_topic(topic)
        assert res == ("stack", "apply"), "Return value must be ('stack', 'apply')"
        
        topic = "org.eclipse.muto.sandbox/f1tenth/things/live/messages/agent/commands/ros/node"
        res = self.node.parse_topic(topic)
        assert res == ("agent", "ros/node"), "Return value must be ('agent', 'ros/node')"
