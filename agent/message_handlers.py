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

"""
Message handlers for different types of messages in the Muto Agent system.
"""

import json
from typing import Any, Dict

from muto_msgs.msg import Gateway, MutoAction
from rclpy.node import Node

from .interfaces import MessageHandler
from .topic_parser import MutoTopicParser
from .config import TopicConfig
from .exceptions import MessageParsingError


class GatewayMessageHandler(MessageHandler):
    """
    Handler for messages received from the gateway.
    
    This handler processes messages from the gateway and routes them
    to the appropriate components based on the message type and content.
    """
    
    def __init__(self, node: Node, topic_parser: MutoTopicParser, config: TopicConfig):
        """
        Initialize the gateway message handler.
        
        Args:
            node: The ROS node instance.
            topic_parser: Topic parser for extracting message information.
            config: Topic configuration.
        """
        self._node = node
        self._topic_parser = topic_parser
        self._config = config
    
    def handle_message(self, data: Gateway) -> None:
        """
        Handle a gateway message.
        
        Routes messages received from gateway to the respective module.
        
        Args:
            data: Gateway message.
        """
        try:
            # Parse message data
            topic = data.topic
            payload = data.payload
            meta = data.meta
            
            # Parse topic to determine message type and method
            message_type, method = self._topic_parser.parse_topic(topic)
            
            # Route message based on type
            if message_type == "ping":
                self._handle_ping_message(payload, meta)
            elif message_type == "stack":
                self._handle_stack_message(payload, meta, method)
            elif message_type == "agent":
                self._handle_agent_message(payload, meta, method)
            else:
                self._node.get_logger().debug(f"Unhandled message type: {message_type}")
                
        except Exception as e:
            self._node.get_logger().error(f"Failed to handle gateway message: {e}")
            raise MessageParsingError(f"Gateway message handling failed: {e}") from e
    
    def _handle_ping_message(self, payload: str, meta: Any) -> None:
        """
        Handle ping messages by responding with pong.
        
        Args:
            payload: Message payload.
            meta: Message metadata.
        """
        try:
            response_payload = payload.replace("/inbox", "/outbox")
            
            msg = Gateway()
            msg.topic = ""
            msg.payload = response_payload
            msg.meta = meta
            
            # Get publisher from node
            pub_gateway = getattr(self._node, '_pub_dict', {}).get('gateway')
            if pub_gateway:
                pub_gateway.publish(msg)
                self._node.get_logger().debug(f"Published ping response message")
            else:
                self._node.get_logger().error("Gateway publisher not available")
                
        except Exception as e:
            self._node.get_logger().error(f"Failed to handle ping message: {e}")
    
    def _handle_stack_message(self, payload: str, meta: Any, method: str) -> None:
        """
        Handle messages destined for the stack/composer.
        
        Args:
            payload: Message payload.
            meta: Message metadata.
            method: Command method to execute.
        """
        try:
            msg_action = MutoAction()
            msg_action.context = ""
            msg_action.method = method
            msg_action.payload = payload
            msg_action.meta = meta
            
            # Get publisher from node
            pub_stack = getattr(self._node, '_pub_dict', {}).get('stack')
            if pub_stack:
                pub_stack.publish(msg_action)
                self._node.get_logger().debug(f"Stack message sent with method: {method}")
            else:
                self._node.get_logger().error("Stack publisher not available")
                
        except Exception as e:
            self._node.get_logger().error(f"Failed to handle stack message: {e}")
    
    def _handle_agent_message(self, payload: str, meta: Any, method: str) -> None:
        """
        Handle messages destined for agent commands.
        
        Args:
            payload: Message payload.
            meta: Message metadata.
            method: Command method to execute.
        """
        try:
            msg_action = MutoAction()
            msg_action.context = ""
            msg_action.method = method
            msg_action.payload = payload
            msg_action.meta = meta
            
            # Get publisher from node
            pub_commands = getattr(self._node, '_pub_dict', {}).get('commands')
            if pub_commands:
                pub_commands.publish(msg_action)
                self._node.get_logger().debug(f"Agent command sent with method: {method}")
            else:
                self._node.get_logger().error("Commands publisher not available")
                
        except Exception as e:
            self._node.get_logger().error(f"Failed to handle agent message: {e}")


class ComposerMessageHandler(MessageHandler):
    """
    Handler for messages from the composer/stack.
    
    Currently a placeholder for future composer message handling functionality.
    """
    
    def __init__(self, node: Node, config: TopicConfig):
        """
        Initialize the composer message handler.
        
        Args:
            node: The ROS node instance.
            config: Topic configuration.
        """
        self._node = node
        self._config = config
    
    def handle_message(self, data: Any) -> None:
        """
        Handle a composer message.
        
        Args:
            data: The message data from composer.
        """
        # TODO: Implement composer message handling
        self._node.get_logger().debug("Composer message received (not implemented)")


class CommandMessageHandler(MessageHandler):
    """
    Handler for messages from command processors.
    
    This handler processes responses and results from command execution
    and routes them back to the appropriate destinations.
    """
    
    def __init__(self, node: Node, config: TopicConfig):
        """
        Initialize the command message handler.
        
        Args:
            node: The ROS node instance.
            config: Topic configuration.
        """
        self._node = node
        self._config = config
    
    def handle_message(self, data: MutoAction) -> None:
        """
        Handle a command response message.
        
        Routes messages received from command processors back to the gateway.
        
        Args:
            data: MutoAction message from command processor.
        """
        try:
            msg = Gateway()
            msg.topic = ""
            msg.payload = data.payload
            msg.meta = data.meta
            
            # Get publisher from node
            pub_gateway = getattr(self._node, '_pub_dict', {}).get('gateway')
            if pub_gateway:
                pub_gateway.publish(msg)
                self._node.get_logger().debug("Command response forwarded to gateway")
            else:
                self._node.get_logger().error("Gateway publisher not available")
                
        except Exception as e:
            self._node.get_logger().error(f"Failed to handle command message: {e}")
            raise MessageParsingError(f"Command message handling failed: {e}") from e