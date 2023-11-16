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

import rclpy

import agent.mqtt
from muto_msgs.msg import Gateway


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


    @patch("paho.mqtt.properties.Properties")
    @patch("paho.mqtt.client.MQTTMessage")
    def test_message_publish(self, mqtt_message, mqtt_message_properties):
        mqtt_message_properties.ResponseTopic = "db-org.eclipse.muto.sandbox:f1tenth/agent/fed4991b-b88f-49d1-aa71-b829f0814cde"
        mqtt_message_properties.CorrelationData = b"fed4991b-b88f-49d1-aa71-b829f0814cde"

        mqtt_message.topic = "org.eclipse.muto.sandbox:f1tenth/stack/commands/apply"
        mqtt_message.payload = b"{'stackId':'org.eclipse.muto.sandbox:f1tenth_melodic_base.launch','target':{'topic':'db-org.eclipse.muto.sandbox:f1tenth/agent/fed4991b-b88f-49d1-aa71-b829f0814cde','correlation':'fed4991b-b88f-49d1-aa71-b829f0814cde'}}"
        mqtt_message.qos = ""
        mqtt_message.retain = ""
        mqtt_message.mid = ""
        mqtt_message.properties = mqtt_message_properties


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

