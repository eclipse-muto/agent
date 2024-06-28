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
from unittest.mock import patch
import json

import rclpy

import agent.mqtt
from muto_msgs.msg import Gateway, Thing


class TestMQTTNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def setUp(self):
        self.node = agent.mqtt.MQTT()

    def tearDown(self):
        self.node.destroy_node()


    def test_mqtt_node_create(self):
        assert self.node != None, "Node couldn't be created."

    # agent message*
    # stack message*
    # thing message*
    # error message
    @patch("paho.mqtt.client.MQTTMessage")
    def test_message_publish_agent_msg(self, mqtt_message):
        mqtt_message.topic = "muto/org.eclipse.muto.sandbox:f1tenth"
        mqtt_message.payload = json.dumps(
            {
                "topic": "org.eclipse.muto.sandbox/f1tenth/things/live/messages/agent/commands/ros/node",
                "headers": {
                    "content-type": "application/json",
                    "reply-to": "muto/org.eclipse.muto.sandbox:f1tenth",
                    "correlation-id": "952f6b77-1d77-4555-a277-1b5bc3d6b6d6"
                },
                "path": "/inbox/messages/agent/commands/ros/node",
                "value": {}
            }
        ).encode("utf-8")
        mqtt_message.qos = ""
        mqtt_message.retain = ""
        mqtt_message.mid = ""


        self.node.received_message = None
        self.node.create_subscription(
            Gateway,
            self.node.get_parameter("gateway_to_agent_topic").value,
            lambda msg: setattr(self.node, "received_message", msg),
            10
        )   

        self.node.on_message("client", "client_data", mqtt_message)

        rclpy.spin_once(self.node, timeout_sec=3)

        assert type(self.node.received_message) == Gateway, "Problem sending Muto Gateway messages."
        assert self.node.received_message.payload == mqtt_message.payload.decode("utf-8"), "Message changed."

    @patch("paho.mqtt.client.MQTTMessage")
    def test_message_publish_stack_msg(self, mqtt_message):
        mqtt_message.topic = "muto/org.eclipse.muto.sandbox:f1tenth"
        mqtt_message.payload = json.dumps(
            {
                "topic": "org.eclipse.muto.sandbox/f1tenth/things/live/messages/stack/commands/apply",
                "headers": {
                    "content-type": "application/json",
                    "reply-to": "muto/org.eclipse.muto.sandbox:f1tenth",
                    "correlation-id": "b4cbf2d5-1a83-433e-94db-a8369d24a42b"
                },
                "path": "/inbox/messages/stack/commands/apply",
                "value": {
                    "stackId": "org.eclipse.muto.sandbox:composable_client_server"
                }
            }
        ).encode("utf-8")
        mqtt_message.qos = ""
        mqtt_message.retain = ""
        mqtt_message.mid = ""


        self.node.received_message = None
        self.node.create_subscription(
            Gateway,
            self.node.get_parameter("gateway_to_agent_topic").value,
            lambda msg: setattr(self.node, "received_message", msg),
            10
        )   

        self.node.on_message("client", "client_data", mqtt_message)

        rclpy.spin_once(self.node, timeout_sec=3)

        assert type(self.node.received_message) == Gateway, "Problem sending Muto Gateway messages."
        assert self.node.received_message.payload == mqtt_message.payload.decode("utf-8"), "Message changed."

    @patch("paho.mqtt.client.MQTTMessage")
    def test_message_publish_twin_msg(self, mqtt_message):
        mqtt_message.topic = "muto/org.eclipse.muto.sandbox:f1tenth"
        mqtt_message.payload = json.dumps(
            {
                "topic": "org.eclipse.muto.sandbox/f1tenth/things/twin/events/modified",
                "headers": {
                    "content-type": "application/json",
                    "correlation-id": "778ebbbb-ee5b-428f-a522-33651aa96e15"
                },
                "path": "/features/f1tenth/properties/vehicle_info/vehicle_info",
                "value": {}
            }
        ).encode("utf-8")
        mqtt_message.qos = ""
        mqtt_message.retain = ""
        mqtt_message.mid = ""


        self.node.received_message = None
        self.node.create_subscription(
            Thing,
            self.node.get_parameter("thing_messages_topic").value,
            lambda msg: setattr(self.node, "received_message", msg),
            10
        )   

        self.node.on_message("client", "client_data", mqtt_message)

        rclpy.spin_once(self.node, timeout_sec=3)

        assert type(self.node.received_message) == Thing, "Problem sending Thing message."
        assert self.node.received_message.path == json.loads(mqtt_message.payload.decode("utf-8"))["path"], "Message changed."
