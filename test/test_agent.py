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

    def tearDown(self):
        self.node.destroy_node()

    def test_mqtt_node_create(self):
        assert self.node != None, "Node couldn't be created."

    def test_gateway_msg_callback_stack(self):
        meta_msg = MutoActionMeta()
        meta_msg.response_topic = "db-org.eclipse.muto.sandbox:f1tenth/agent/fa30fe44-153f-4541-b78e-3f71863a331d"
        meta_msg.correlation_data = "fa30fe44-153f-4541-b78e-3f71863a331d"

        gw_msg = Gateway()
        gw_msg.topic = "org.eclipse.muto.sandbox:f1tenth/stack/commands/apply"
        gw_msg.payload = '{"stackId":"org.eclipse.muto.sandbox:f1tenth_melodic_base.launch","target":{"topic":"db-org.eclipse.muto.sandbox:f1tenth/agent/fed4991b-b88f-49d1-aa71-b829f0814cde","correlation":"fed4991b-b88f-49d1-aa71-b829f0814cde"}}'
        gw_msg.meta = meta_msg

        self.node.received_message = None
        self.node.create_subscription(
            MutoAction,
            self.node.get_parameter("stack_topic").value,
            lambda msg: setattr(self.node, "received_message", msg),
            10
        )

        self.node.gateway_msg_callback(gw_msg)

        rclpy.spin_once(self.node, timeout_sec=3)

        assert type(self.node.received_message) == MutoAction
        assert self.node.received_message.context == ""
        assert self.node.received_message.method == "apply"
        assert self.node.received_message.payload == gw_msg.payload
        assert self.node.received_message.meta == meta_msg

    def test_gateway_msg_callback_agent(self):
        meta_msg = MutoActionMeta()
        meta_msg.response_topic = "db-org.eclipse.muto.sandbox:f1tenth/agent/5e093972-082a-44bf-854b-96c36b3194f0"
        meta_msg.correlation_data = "5e093972-082a-44bf-854b-96c36b3194f0"

        gw_msg = Gateway()
        gw_msg.topic = "org.eclipse.muto.sandbox:f1tenth/agent/commands/ros/node"
        gw_msg.payload = '{"target":{"topic":"db-org.eclipse.muto.sandbox:f1tenth/agent/5e093972-082a-44bf-854b-96c36b3194f0","correlation":"5e093972-082a-44bf-854b-96c36b3194f0"}}'
        gw_msg.meta = meta_msg

        self.node.received_message = None
        self.node.create_subscription(
            MutoAction,
            self.node.get_parameter("agent_to_commands_topic").value,
            lambda msg: setattr(self.node, "received_message", msg),
            10
        )

        self.node.gateway_msg_callback(gw_msg)

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

        self.node.commands_msg_callback(action_msg)

        rclpy.spin_once(self.node, timeout_sec=3)

        assert type(self.node.received_message) == Gateway
        assert self.node.received_message.topic == ""
        assert self.node.received_message.payload == action_msg.payload
        assert self.node.received_message.meta == meta_msg

    def test_parse_topic(self):
        topic = "org.eclipse.muto.sandbox:f1tenth/stack/commands/apply"
        res = self.node.parse_topic(topic)
        assert res == ("stack", "apply"), "Return value must be ('stack', 'apply')"
        
        topic = "org.eclipse.muto.sandbox:f1tenth/agent/commands/foo/bar"
        res = self.node.parse_topic(topic)
        assert res == ("agent", "foo/bar"), "Return value must be ('agent', 'foo/bar')"

        topic = "org.eclipse.muto.sandbox:/commands/foo/bar"
        res = self.node.parse_topic(topic)
        assert res == (None, None), "Return value must be (None, None)"
