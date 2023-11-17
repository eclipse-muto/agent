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

from muto_msgs.msg import Gateway, MutoActionMeta

from paho.mqtt.client import Client, MQTTv5
from paho.mqtt.properties import Properties
from paho.mqtt.packettypes import PacketTypes

class MQTT(Node):

    def __init__(self):
        super().__init__("mqtt_gateway")

        # Declare Parameters
        self.declare_parameter("host", "sandbox.composiv.ai")
        self.declare_parameter("port", 1883)
        self.declare_parameter("keep_alive", 60)
        self.declare_parameter("user", "")
        self.declare_parameter("password", "")
        self.declare_parameter("namespace", "")
        self.declare_parameter("name", "")

        self.declare_parameter("agent_to_gateway_topic", "msg1")
        self.declare_parameter("gateway_to_agent_topic", "msg2")

        # Initialize Parameters
        self.host = self.get_parameter("host").value
        self.port = self.get_parameter("port").value
        self.keep_alive = self.get_parameter("keep_alive").value
        self.user = self.get_parameter("user").value
        self.password = self.get_parameter("password").value
        self.namespace = self.get_parameter("namespace").value
        self.name = self.get_parameter("name").value

        self.agent_to_gateway_topic = self.get_parameter("agent_to_gateway_topic").value
        self.gateway_to_agent_topic = self.get_parameter("gateway_to_agent_topic").value

        # MQTT Client
        self.mqtt = Client(
            client_id=f"{self.name}_{self.get_clock().now()}",
            reconnect_on_failure=True,
            protocol=MQTTv5
        )
        self.mqtt.on_connect = self.on_connect
        self.mqtt.on_message = self.on_message

        try:
            self.mqtt.connect(self.host, self.port, self.keep_alive)
            self.get_logger().info("Connection Established!")
        except:
            self.get_logger().warn("Connection could not be established to Twin Server!")

        self.mqtt.loop_start()


        # ROS Related
        self.pub_agent = self.create_publisher(Gateway, self.gateway_to_agent_topic, 10)
        self.sub_agent = self.create_subscription(Gateway, self.agent_to_gateway_topic, self.agent_msg_callback, 10)


    def __del__(self):
        self.mqtt.loop_stop()
        self.mqtt.disconnect()

    def on_connect(self, client, userdata, flags, reasonCode, properties):
        """
        Connect callback implementation.

        This method runs everytime there is a new connection. Since subscribing
        occurs here, even if we lose connection and reconnect, subscription
        will be renewed.
            
        Args:
            client:
                The client instance for this callback
            userdata:
                The private user data as set in Client() or userdata_set()
            flags:
                Response flags sent by the broker
            reasonCode:
                The MQTT v5.0 reason code: an instance of the ReasonCode class.
                ReasonCode may be compared to integer.
            properties:
                The MQTT v5.0 properties returned from the broker. An instance
                of the Properties class.
        """
        topic = f"{self.namespace}:{self.name}/#"
        self.mqtt.subscribe(topic)
        self.get_logger().info(f"Subscribed to {topic}")

    def on_message(self, client, userdata, message):
        """
        Message callback implementation.

        This method runs everytime there is a new message. Parses the new
        message. Creates a dict name "meta" if there is ResponseTopic and
        CorrelationData in message properties. Sends processed "message
        topic", "message payload" and "meta" to agent.
        

        Args:
            client:
                The client instance for this callback.
            userdata:
                The private user data as set in Client() or userdata_set().
            message:
                An instance of MQTTMessage. This is a class with members
                topic, payload, qos, retain, mid and properties.
        """
        # Parse message
        topic = message.topic
        payload = message.payload.decode("utf-8")
        qos = message.qos
        retain = message.retain
        mid = message.mid
        properties = message.properties

        # Create "meta" if ResponseTopic and CorrelationData exists in message properties
        ResponseTopic = None
        CorrelationData = None
        meta = MutoActionMeta()

        if hasattr(properties, "ResponseTopic") and hasattr(properties, "CorrelationData"):
            ResponseTopic = properties.ResponseTopic
            CorrelationData = properties.CorrelationData.decode("utf-8")

            meta.response_topic = ResponseTopic
            meta.correlation_data = CorrelationData

        # Send message to agent
        self.send_to_agent(topic, payload, meta)

    def send_to_agent(self, topic, payload, meta):
        """
        Construct and publish Muto Gateway message.

        Args:
            topic: Parsed mqtt message topic.
            payload: Parsed mqtt message payload.
            meta: Meta string.
        """
        msg = Gateway()
        msg.topic = topic
        msg.payload = payload
        msg.meta = meta
        
        self.pub_agent.publish(msg)

    def agent_msg_callback(self, data):
        """
        Callback function of agent subscriber.

        Publishes messages received from agent to ditto server.
        
        Args:
            data: Gateway message.
        """
        payload = data.payload
        response_topic = data.meta.response_topic
        correlation_data = data.meta.correlation_data

        properties = Properties(PacketTypes.PUBLISH)
        properties.CorrelationData = correlation_data.encode()

        self.mqtt.publish(response_topic, payload, properties=properties)


def main():
    rclpy.init()
    mqtt = MQTT()
    rclpy.spin(mqtt)


if __name__ == '__main__':
    main()
