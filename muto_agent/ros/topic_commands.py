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

# Standard library imports
import json
import time
from typing import Any, Dict, Type, Optional

# Third-party imports
from ackermann_msgs.msg import *
from diagnostic_msgs.msg import *
from geometry_msgs.msg import *
from nav_msgs.msg import *
from sensor_msgs.msg import *
from std_msgs.msg import *
from tf2_msgs.msg import *
from rclpy.node import Node
from muto_msgs.msg import *
from muto_msgs.srv import CommandPlugin, CoreTwin

# Local imports
from muto_agent.ros.msg_converter import json_message_converter


class TopicEcho:
    """
    Handles topic echoing functionality for ROS 2 topics.
    
    This class provides functionality to subscribe to ROS topics and 
    echo their messages at specified rates, with support for various
    message types and conversion to JSON format.
    
    Args:
        node: The ROS node instance for creating subscriptions.
        payload: Configuration payload containing topic and rate info.
        topic_type: The ROS message type for the topic.
        meta: Message metadata for response handling.
    """

    def __init__(
        self, 
        node: Node, 
        payload: Dict[str, Any], 
        topic_type: str, 
        meta: Any
    ) -> None:
        self.node = node
        
        self.payload = payload
        self.ros_topic = payload["topic"]
        self.action = payload["action"]
        # Divide by 1000 to convert ms to s
        self.rate = payload["rate"]/1000
        self.target = payload["topic"]
        self.meta = meta
        
        self.ros_topic_type = self.convert_ros_topic_type(topic_type)

        self.sub = None
        self.last_send = time.time()

    def convert_ros_topic_type(self, topic_type: str) -> Type[Any]:
        """
        Converts the ROS topic type string into its corresponding Python class.

        Args:
            topic_type (str): The ROS topic type string.

        Returns:
            type: The Python class representing the converted topic type.

        Raises:
            KeyError: If the topic type string is not found in the available imports.

        Example:
            If `topic_type` is 'std_msgs.msg.String', this method will return the
            corresponding Python class `std_msgs.msg.String`.

        """
        imports = globals().copy()
        try:
            converted_topic_type = imports[topic_type]
        except KeyError:
            self.node.get_logger().error("Requested topic type is not valid.")

        return converted_topic_type

    def register_telemetry(self):
        client = self.node.create_client(CoreTwin, "core_twin/register_telemetry")
        req = CoreTwin.Request()
        req.input = json.dumps(self.payload)
        client.call_async(req)

    def delete_telemetry(self):
        client = self.node.create_client(CoreTwin, "core_twin/delete_telemetry")
        req = CoreTwin.Request()
        req.input = json.dumps(self.payload)
        client.call_async(req)

    def start(self):
        self.register_telemetry()
        self.subscribe()
    
    def stop(self):
        if self.sub != None:
            self.node.destroy_subscription(self.sub)
            self.sub = None

    def delete(self):
        self.stop()
        self.delete_telemetry()


    def subscribe(self):
        self.sub = self.node.create_subscription(
            self.ros_topic_type,
            self.ros_topic,
            self.topic_callback,
            1
        )
    
    def topic_callback(self, data):
        message_time = time.time() 

        if message_time-self.last_send > self.rate:
            self.last_send = message_time
            msg = json_message_converter.convert_ros_message_to_json(data)

            self.node.publish_telemetry(msg, self.meta)


class TopicCommands():
    """ # TODO add docs."""

    def __init__(self, node):
        self.node = node

        self.echoed_nodes = {}

        self.rostopic_list = self.node.create_service(CommandPlugin, "rostopic_list", self.callback_rostopic_list)
        self.rostopic_info = self.node.create_service(CommandPlugin, "rostopic_info", self.callback_rostopic_info)
        self.rostopic_echo = self.node.create_service(CommandPlugin, "rostopic_echo", self.callback_rostopic_echo)

    def callback_rostopic_list(self, request, response):
        """
        Callback function for handling rostopic_list service request.

        This function retrieves information about discovered topics and
        constructs a response containing the information about publishing and
        subscribing nodes.

        Args:
            request: The request message object.
            response: The response message object.

        Returns:
            The modified response message object.
        """
        topics = self.node.get_topic_names_and_types()
        pubs = self.construct_publishers(topics)
        subs = self.construct_subscribers(topics)

        result = {"pubs": pubs, "subs": subs}


        response.output = self.node.construct_command_output_message(result)

        return response
    
    def callback_rostopic_info(self, request, response):
        """
        Callback function for handling rostopic_info service request.

        This function retreives information about a specific node and
        constructs a response containing the information about publishing and
        subscribing nodes.

        Args:
            request: The request message object.
            response: The response message object.

        Returns:
            The modified response message object.
        """
        payload = json.loads(request.input.payload)
        result = {"status": "NOTFOUND"}

        requested_topic = payload.get("topic", None)

        if requested_topic != None:
            result = self.get_topic_info(requested_topic)


        response.output = self.node.construct_command_output_message(result)

        return response

    def callback_rostopic_echo(self, request, response):
        """ # TODO add docs."""
        payload = json.loads(request.input.payload)["value"]
        ros_topic_to_echo = payload["topic"]
        action = payload["action"]
        rate = payload["rate"]
        meta = payload["target"]

        topics = self.node.get_topic_names_and_types()

        if ros_topic_to_echo in [item[0] for item in topics]:
            try:
                topic_type = self.get_topic_type(topics, ros_topic_to_echo)

                if action == "start":
                    if ros_topic_to_echo not in self.echoed_nodes:
                        topic_echo = TopicEcho(self.node, payload, topic_type, meta)
                        self.echoed_nodes[ros_topic_to_echo] = topic_echo
                    echoed_node = self.echoed_nodes[ros_topic_to_echo]
                    echoed_node.start()
                    status = {"status": "STARTED", "topic": ros_topic_to_echo}

                elif action == "stop":
                    if ros_topic_to_echo not in self.echoed_nodes:
                        topic_echo = TopicEcho(self.node, payload, topic_type, meta)
                        self.echoed_nodes[ros_topic_to_echo] = topic_echo
                    echoed_node = self.echoed_nodes[ros_topic_to_echo]
                    echoed_node.stop()
                    status = {"status": "STOPPED", "topic": ros_topic_to_echo}

                elif action == "delete":
                    if ros_topic_to_echo not in self.echoed_nodes:
                        topic_echo = TopicEcho(self.node, payload, topic_type, meta)
                        self.echoed_nodes[ros_topic_to_echo] = topic_echo
                    echoed_node = self.echoed_nodes[ros_topic_to_echo]
                    echoed_node.delete()
                    if self.echoed_nodes != None:
                        self.echoed_nodes.pop(ros_topic_to_echo)
                    status = {"status": "DELETED", "topic": ros_topic_to_echo}

                elif action == "register":
                    if ros_topic_to_echo not in self.echoed_nodes:
                        topic_echo = TopicEcho(self.node, payload, topic_type, meta)
                        self.echoed_nodes[ros_topic_to_echo] = topic_echo
                    topic_echo = self.echoed_nodes[ros_topic_to_echo]
                    topic_echo.register_telemetry()
                    status = {"status": "REGISTERED", "topic": ros_topic_to_echo}

            except Exception as e:
                status = {"status": "EXCEPTION", "exception": str(e)}

        else:
            status = {"status": "NOTFOUND", "topic": ros_topic_to_echo}

        response.output = self.node.construct_command_output_message(status)
        return response

    def construct_publishers(self, topics):
        """
        Constructs a list of publishers based on the topics.

        Args:
            topics (list[tuple]): A list of tuples representing the topics and
                topic types. Each tuple should have the following structure:
                (topic, type).

        Returns:
            A list of tuples containing the topic, type, and a list of
            publishers.
        """
        pubs = []

        for topic, type_ in topics:
            endpoints = self.node.get_publishers_info_by_topic(topic)

            temp = []
            for endpoint in endpoints:
                temp.append(endpoint.node_name)
            
            pubs.append((topic, type_, temp))
        
        return pubs

    def construct_subscribers(self, topics):
        """
        Constructs a list of subscribers based on the topics.

        Args:
            topics (list[tuple]): A list of tuples representing the topics and
                topic types. Each tuple should have the following structure:
                (topic, type).

        Returns:
            A list of tuples containing the topic, type, and a list of
            subscribers.
        """
        subs = []

        for topic, type_ in topics:
            endpoints = self.node.get_subscriptions_info_by_topic(topic)

            temp = []
            for endpoint in endpoints:
                temp.append(endpoint.node_name)
            
            subs.append((topic, type_, temp))
        
        return subs

    def get_topic_info(self, topic):
        """
        Retrieves information about a specific topic.

        Args:
            topic (str): The name of the topic to retrieve information for.

        Returns:
            A dictionary containing the topic information.
        """
        info = {"topic": topic, "types": None, "pubs": [], "subs": []}

        publishers = self.node.get_publishers_info_by_topic(topic)
        subscriptions = self.node.get_subscriptions_info_by_topic(topic)

        for publisher in publishers:
            info["pubs"].append({"publisher": publisher.node_name})
            if info["types"] == None:
                info["types"] = publisher.topic_type

        for subscription in subscriptions:
            info["subs"].append({"subscriber": subscription.node_name})
            if info["types"] == None:
                info["types"] = subscription.topic_type
        
        return info
    
    def get_topic_type(self, topics, requested_topic):
        for topic, type_ in topics:
            if topic == requested_topic:
                type_ = type_[0].split("/")[-1]
                return type_
