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
Configuration management for the Muto Agent system.
"""

from typing import Dict, Any, Optional
from dataclasses import dataclass, field
import rclpy
from rclpy.node import Node

from .exceptions import ConfigurationError


@dataclass
class MQTTConfig:
    """Configuration for MQTT connection."""
    host: str = "sandbox.composiv.ai"
    port: int = 1883
    keep_alive: int = 60
    user: str = ""
    password: str = ""
    namespace: str = ""
    prefix: str = "muto"
    name: str = ""


@dataclass
class TopicConfig:
    """Configuration for ROS topics."""
    stack_topic: str = "stack"
    twin_topic: str = "twin"
    agent_to_gateway_topic: str = "agent_to_gateway"
    gateway_to_agent_topic: str = "gateway_to_agent"
    agent_to_commands_topic: str = "agent_to_command"
    commands_to_agent_topic: str = "command_to_agent"
    thing_messages_topic: str = "thing_messages"


@dataclass
class AgentConfig:
    """Main configuration for the Muto Agent."""
    mqtt: MQTTConfig = field(default_factory=MQTTConfig)
    topics: TopicConfig = field(default_factory=TopicConfig)


class ConfigurationManager:
    """
    Manages configuration loading and validation for the Muto Agent system.
    
    This class centralizes all configuration management, providing a clean
    interface for accessing configuration parameters with proper validation
    and error handling.
    """
    
    def __init__(self, node: Node):
        """
        Initialize the configuration manager.
        
        Args:
            node: The ROS node to read parameters from.
        """
        self._node = node
        self._config: Optional[AgentConfig] = None
        
    def load_config(self) -> AgentConfig:
        """
        Load configuration from ROS parameters.
        
        Returns:
            AgentConfig: The loaded configuration.
            
        Raises:
            ConfigurationError: If configuration loading fails.
        """
        try:
            # Declare all parameters with defaults
            self._declare_parameters()
            
            # Load MQTT configuration
            mqtt_config = MQTTConfig(
                host=self._get_parameter("host", "sandbox.composiv.ai"),
                port=self._get_parameter("port", 1883),
                keep_alive=self._get_parameter("keep_alive", 60),
                user=self._get_parameter("user", ""),
                password=self._get_parameter("password", ""),
                namespace=self._get_parameter("namespace", ""),
                prefix=self._get_parameter("prefix", "muto"),
                name=self._get_parameter("name", "")
            )
            
            # Load topic configuration
            topic_config = TopicConfig(
                stack_topic=self._get_parameter("stack_topic", "stack"),
                twin_topic=self._get_parameter("twin_topic", "twin"),
                agent_to_gateway_topic=self._get_parameter("agent_to_gateway_topic", "agent_to_gateway"),
                gateway_to_agent_topic=self._get_parameter("gateway_to_agent_topic", "gateway_to_agent"),
                agent_to_commands_topic=self._get_parameter("agent_to_commands_topic", "agent_to_command"),
                commands_to_agent_topic=self._get_parameter("commands_to_agent_topic", "command_to_agent"),
                thing_messages_topic=self._get_parameter("thing_messages_topic", "thing_messages")
            )
            
            self._config = AgentConfig(mqtt=mqtt_config, topics=topic_config)
            self._validate_config()
            
            self._node.get_logger().info("Configuration loaded successfully")
            return self._config
            
        except Exception as e:
            raise ConfigurationError(f"Failed to load configuration: {e}") from e
    
    def get_config(self) -> AgentConfig:
        """
        Get the current configuration.
        
        Returns:
            AgentConfig: The current configuration.
            
        Raises:
            ConfigurationError: If configuration has not been loaded.
        """
        if self._config is None:
            raise ConfigurationError("Configuration not loaded. Call load_config() first.")
        return self._config
    
    def _declare_parameters(self) -> None:
        """Declare all ROS parameters with their default values."""
        parameters = [
            ("host", "sandbox.composiv.ai"),
            ("port", 1883),
            ("keep_alive", 60),
            ("user", ""),
            ("password", ""),
            ("namespace", ""),
            ("prefix", "muto"),
            ("name", ""),
            ("stack_topic", "stack"),
            ("twin_topic", "twin"),
            ("agent_to_gateway_topic", "agent_to_gateway"),
            ("gateway_to_agent_topic", "gateway_to_agent"),
            ("agent_to_commands_topic", "agent_to_command"),
            ("commands_to_agent_topic", "command_to_agent"),
            ("thing_messages_topic", "thing_messages")
        ]
        
        for param_name, default_value in parameters:
            try:
                self._node.declare_parameter(param_name, default_value)
            except rclpy.exceptions.ParameterAlreadyDeclaredException as e:
                self._node.get_logger().warning(f"Parameter {param_name} already declared: {e}")
    
    def _get_parameter(self, name: str, default: Any) -> Any:
        """
        Get a parameter value safely.
        
        Args:
            name: Parameter name.
            default: Default value if parameter is not set.
            
        Returns:
            The parameter value.
        """
        try:
            return self._node.get_parameter(name).value
        except Exception:
            self._node.get_logger().warning(f"Failed to get parameter '{name}', using default: {default}")
            return default
    
    def _validate_config(self) -> None:
        """
        Validate the loaded configuration.
        
        Raises:
            ConfigurationError: If configuration is invalid.
        """
        if not self._config:
            raise ConfigurationError("Configuration is None")
            
        # Validate MQTT configuration
        if self._config.mqtt.port < 1 or self._config.mqtt.port > 65535:
            raise ConfigurationError(f"Invalid MQTT port: {self._config.mqtt.port}")
            
        if self._config.mqtt.keep_alive < 1:
            raise ConfigurationError(f"Invalid MQTT keep_alive: {self._config.mqtt.keep_alive}")
            
        # Validate required fields
        if not self._config.mqtt.host:
            raise ConfigurationError("MQTT host is required")