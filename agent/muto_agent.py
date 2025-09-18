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
Main Muto Agent module providing centralized message routing and coordination.

This module implements the core Muto Agent that acts as a message router
between different components of the system including gateways, composers,
and command processors.
"""

# Standard library imports
from typing import Optional, Tuple

# Third-party imports
import rclpy
from std_msgs.msg import String
from muto_msgs.msg import Gateway, MutoAction

# Local imports
from .interfaces import BaseNode, MessageHandler
from .config import ConfigurationManager, AgentConfig
from .topic_parser import MutoTopicParser
from .message_handlers import (
    GatewayMessageHandler, 
    ComposerMessageHandler, 
    CommandMessageHandler
)
from .exceptions import ConfigurationError, TopicParsingError


class MutoAgent(BaseNode):
    """
    Main Muto Agent class that coordinates message routing between components.
    
    The MutoAgent acts as a central hub for message routing, handling communication
    between gateways, composers, and command processors. It uses a modular design
    with separate message handlers for different message types.
    
    Features:
    - Centralized configuration management
    - Robust error handling with specific exception types
    - Modular message handling through dedicated handlers
    - Proper resource management and cleanup
    - Comprehensive logging
    """

    def __init__(self):
        """Initialize the Muto Agent."""
        super().__init__("muto_agent")
        
        self._config_manager: Optional[ConfigurationManager] = None
        self._config: Optional[AgentConfig] = None
        self._topic_parser: Optional[MutoTopicParser] = None
        
        # Message handlers
        self._gateway_handler: Optional[GatewayMessageHandler] = None
        self._composer_handler: Optional[ComposerMessageHandler] = None
        self._command_handler: Optional[CommandMessageHandler] = None
        
        # Publishers and subscribers (using different names to avoid ROS conflicts)
        self._pub_dict = {}
        self._sub_dict = {}

    def _do_initialize(self) -> None:
        """Initialize the agent components."""
        try:
            # Initialize configuration
            self._config_manager = ConfigurationManager(self)
            self._config = self._config_manager.load_config()
            
            # Initialize topic parser
            self._topic_parser = MutoTopicParser(self.get_logger())
            
            # Initialize message handlers
            self._initialize_message_handlers()
            
            # Setup ROS communication
            self._setup_ros_communication()
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize MutoAgent: {e}")
            raise ConfigurationError(f"Agent initialization failed: {e}") from e

    def _initialize_message_handlers(self) -> None:
        """Initialize all message handlers."""
        self._gateway_handler = GatewayMessageHandler(
            self, self._topic_parser, self._config.topics
        )
        self._composer_handler = ComposerMessageHandler(
            self, self._config.topics
        )
        self._command_handler = CommandMessageHandler(
            self, self._config.topics
        )

    def _setup_ros_communication(self) -> None:
        """Setup ROS publishers and subscribers."""
        topics = self._config.topics
        
        # Setup publishers
        self._pub_dict['gateway'] = self.create_publisher(
            Gateway, topics.agent_to_gateway_topic, 10
        )
        self._pub_dict['stack'] = self.create_publisher(
            MutoAction, topics.stack_topic, 10
        )
        self._pub_dict['commands'] = self.create_publisher(
            MutoAction, topics.agent_to_commands_topic, 10
        )
        
        # Setup subscribers
        self._sub_dict['gateway'] = self.create_subscription(
            Gateway, topics.gateway_to_agent_topic, self._gateway_msg_callback, 10
        )
        self._sub_dict['stack'] = self.create_subscription(
            String, topics.twin_topic, self._composer_msg_callback, 10
        )
        self._sub_dict['commands'] = self.create_subscription(
            MutoAction, topics.commands_to_agent_topic, self._commands_msg_callback, 10
        )

    def _gateway_msg_callback(self, data: Gateway) -> None:
        """
        Callback function for gateway subscriber.
        
        Args:
            data: Gateway message.
        """
        try:
            if self._gateway_handler:
                self._gateway_handler.handle_message(data)
            else:
                self.get_logger().error("Gateway handler not initialized")
        except Exception as e:
            self.get_logger().error(f"Failed to process gateway message: {e}")

    def _composer_msg_callback(self, data: String) -> None:
        """
        Callback function for composer subscriber.
        
        Args:
            data: String message from composer.
        """
        try:
            if self._composer_handler:
                self._composer_handler.handle_message(data)
            else:
                self.get_logger().debug("Composer handler not initialized")
        except Exception as e:
            self.get_logger().error(f"Failed to process composer message: {e}")

    def _commands_msg_callback(self, data: MutoAction) -> None:
        """
        Callback function for commands subscriber.
        
        Args:
            data: MutoAction message from commands.
        """
        try:
            if self._command_handler:
                self._command_handler.handle_message(data)
            else:
                self.get_logger().error("Command handler not initialized")
        except Exception as e:
            self.get_logger().error(f"Failed to process command message: {e}")

    def _do_cleanup(self) -> None:
        """Clean up agent resources."""
        try:
            # Clean up subscribers
            for sub in self._sub_dict.values():
                if sub:
                    self.destroy_subscription(sub)
            self._sub_dict.clear()
            
            # Clean up publishers
            for pub in self._pub_dict.values():
                if pub:
                    self.destroy_publisher(pub)
            self._pub_dict.clear()
            
            self.get_logger().info("Agent cleanup completed")
            
        except Exception as e:
            self.get_logger().error(f"Error during cleanup: {e}")

    def get_topic_parser(self) -> Optional[MutoTopicParser]:
        """
        Get the topic parser instance.
        
        Returns:
            The topic parser instance or None if not initialized.
        """
        return self._topic_parser
    
    def get_config(self) -> Optional[AgentConfig]:
        """
        Get the agent configuration.
        
        Returns:
            The agent configuration or None if not loaded.
        """
        return self._config
    
    def parse_topic(self, topic: str) -> Tuple[Optional[str], Optional[str]]:
        """
        Parse topic using the topic parser.
        
        Args:
            topic: Topic string to parse.
            
        Returns:
            Tuple of (type, method) or (None, None) if parsing fails.
        """
        if self._topic_parser:
            return self._topic_parser.parse_topic(topic)
        return None, None
    
    def is_ready(self) -> bool:
        """
        Check if the agent is fully initialized and ready.
        
        Returns:
            True if agent is ready, False otherwise.
        """
        return (
            self._config is not None
            and self._topic_parser is not None
            and self._gateway_handler is not None
            and self._command_handler is not None
            and len(self._pub_dict) > 0
            and len(self._sub_dict) > 0
        )


def main():
    """Main entry point for the Muto Agent."""
    rclpy.init()
    
    try:
        agent = MutoAgent()
        agent.initialize()
        
        if agent.is_ready():
            agent.get_logger().info("Muto Agent started successfully")
            rclpy.spin(agent)
        else:
            agent.get_logger().error("Muto Agent failed to initialize properly")
            
    except Exception as e:
        print(f"Failed to start Muto Agent: {e}")
        
    finally:
        try:
            agent.cleanup()
        except:
            pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()
