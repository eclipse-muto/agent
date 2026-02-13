#
# Copyright (c) 2023 Composiv.ai
#
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# http://www.eclipse.org/legal/epl-2.0.
#
# SPDX-License-Identifier: EPL-2.0
#
# Contributors:
#   Composiv.ai - initial API and implementation
#

"""
Configuration management for the Muto Agent system.
"""

from dataclasses import dataclass, field
from typing import Any

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
class SymphonyConfig:
    """Configuration for Symphony connection."""

    mqtt: MQTTConfig = field(default_factory=MQTTConfig)
    target: str = "muto-target"
    enabled: bool = False
    topic_prefix: str = "symphony"
    api_url: str = ("http://localhost:8082/v1alpha2/",)
    provider_name: str = ("providers.target.mqtt",)
    broker_address: str = ("tcp://mosquitto:1883",)
    client_id: str = ("symphony",)
    request_topic: str = ("coa-request",)
    response_topic: str = ("coa-response",)
    timeout_seconds: int = (30,)
    auto_register: bool = False


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
    symphony: SymphonyConfig = field(default_factory=SymphonyConfig)


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
        self._config: AgentConfig | None = None

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
                name=self._get_parameter("name", ""),
            )

            sym_mqtt_config = MQTTConfig(
                host=self._get_parameter("symphony_host", "sandbox.composiv.ai"),
                port=self._get_parameter("symphony_port", 1883),
                keep_alive=self._get_parameter("symphony_keep_alive", 60),
                user=self._get_parameter("symphony_user", ""),
                password=self._get_parameter("symphony_password", ""),
                namespace=self._get_parameter("symphony_namespace", ""),
                prefix=self._get_parameter("symphony_prefix", "muto"),
                name=self._get_parameter("symphony_name", ""),
            )

            symphony_config = SymphonyConfig(
                mqtt=sym_mqtt_config,
                target=self._get_parameter("symphony_target_name", "muto-target"),
                enabled=self._get_parameter("symphony_enabled", False),
                topic_prefix=self._get_parameter("symphony_topic_prefix", "symphony"),
                api_url=self._get_parameter("symphony_api_url", "http://localhost:8082/v1alpha2/"),
                provider_name=self._get_parameter("symphony_provider_name", "providers.target.mqtt"),
                broker_address=self._get_parameter("symphony_broker_address", "tcp://mosquitto:1883"),
                client_id=self._get_parameter("symphony_client_id", "symphony"),
                request_topic=self._get_parameter("symphony_request_topic", "coa-request"),
                response_topic=self._get_parameter("symphony_response_topic", "coa-response"),
                timeout_seconds=self._get_parameter("symphony_timeout_seconds", 30),
                auto_register=self._get_parameter("symphony_auto_register", False),
            )

            # Load topic configuration
            topic_config = TopicConfig(
                stack_topic=self._get_parameter("stack_topic", "stack"),
                twin_topic=self._get_parameter("twin_topic", "twin"),
                agent_to_gateway_topic=self._get_parameter("agent_to_gateway_topic", "agent_to_gateway"),
                gateway_to_agent_topic=self._get_parameter("gateway_to_agent_topic", "gateway_to_agent"),
                agent_to_commands_topic=self._get_parameter("agent_to_commands_topic", "agent_to_command"),
                commands_to_agent_topic=self._get_parameter("commands_to_agent_topic", "command_to_agent"),
                thing_messages_topic=self._get_parameter("thing_messages_topic", "thing_messages"),
            )

            self._config = AgentConfig(mqtt=mqtt_config, topics=topic_config, symphony=symphony_config)
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
            ("thing_messages_topic", "thing_messages"),
            ("symphony_enabled", False),
            ("symphony_host", "sandbox.composiv.ai"),
            ("symphony_port", 1883),
            ("symphony_keep_alive", 60),
            ("symphony_namespace", ""),
            ("symphony_prefix", "muto"),
            ("symphony_target_name", "muto-device-001"),
            ("symphony_topic_prefix", "symphony"),
            ("symphony_enable", False),
            ("symphony_api_url", "http://localhost:8082/v1alpha2/"),
            ("symphony_user", "admin"),
            ("symphony_password", ""),
            ("symphony_name", "muto-device-001"),
            ("symphony_provider_name", "providers.target.mqtt"),
            ("symphony_broker_address", "tcp://mosquitto:1883"),
            ("symphony_client_id", "symphony"),
            ("symphony_request_topic", "coa-request"),
            ("symphony_response_topic", "coa-response"),
            ("symphony_timeout_seconds", "30"),
            ("symphony_auto_register", False),
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
