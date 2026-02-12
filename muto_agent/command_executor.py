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
Command execution and management for the Muto Agent system.
"""

from __future__ import annotations

import json
from collections.abc import Callable
from concurrent.futures import Future
from typing import Any

from muto_msgs.msg import CommandInput, CommandOutput, PluginResponse
from muto_msgs.srv import CommandPlugin
from rclpy.client import Client

from .exceptions import CommandNotFoundError, ConfigurationError, ServiceNotReadyError
from .interfaces import BaseNode, CommandExecutor


class Command:
    """
    Represents a single executable command with its associated service.

    This class encapsulates the logic for executing a specific command
    via ROS service calls, providing better error handling and logging.
    """

    def __init__(self, node: BaseNode, service_name: str, plugin_type: type):
        """
        Initialize a command.

        Args:
            node: The ROS node instance.
            service_name: Name of the service to call.
            plugin_type: Type of the plugin service.
        """
        self._node = node
        self._service_name = service_name
        self._plugin_type = plugin_type

        self._client: Client | None = None
        self._initialize_client()

    def _initialize_client(self) -> None:
        """Initialize the service client."""
        try:
            self._client = self._node.create_client(self._plugin_type, self._service_name)
            self._node.get_logger().debug(f"Service client created for {self._service_name}")
        except Exception as e:
            self._node.get_logger().error(
                f"Failed to create service client for {self._service_name}: {e}"
            )
            raise ConfigurationError(f"Service client creation failed: {e}") from e

    def execute(
        self, method: str, payload: str, meta: Any, callback: Callable | None = None
    ) -> Future | None:
        """
        Execute the command asynchronously.

        Args:
            method: The command method to execute.
            payload: Command payload.
            meta: Command metadata.
            callback: Optional callback for handling the result.

        Returns:
            Future object representing the service call, or None if service not ready.

        Raises:
            ServiceNotReadyError: If the service is not ready.
        """
        if not self._client:
            raise ServiceNotReadyError(f"Service client for {self._service_name} not initialized")

        if not self._client.service_is_ready():
            raise ServiceNotReadyError(f"Service {self._service_name} not ready")

        try:
            # Create request
            request = CommandPlugin.Request()
            command_input = CommandInput()
            command_input.command = method
            command_input.payload = payload
            request.input = command_input

            # Make async call
            future = self._client.call_async(request)

            if callback:
                future.add_done_callback(lambda f: callback(f, payload, meta))

            self._node.get_logger().debug(
                f"Command {method} executed on service {self._service_name}"
            )
            return future

        except Exception as e:
            self._node.get_logger().error(f"Failed to execute command {method}: {e}")
            raise

    def cleanup(self) -> None:
        """Clean up command resources."""
        if self._client and self._node:
            try:
                self._node.destroy_client(self._client)
                self._client = None
                self._node.get_logger().debug(f"Service client for {self._service_name} cleaned up")
            except Exception as e:
                self._node.get_logger().error(f"Error cleaning up service client: {e}")


class CommandRegistry:
    """
    Registry for managing available commands.

    This class provides a centralized way to register, lookup, and manage
    commands loaded from configuration.
    """

    def __init__(self, node: BaseNode):
        """
        Initialize the command registry.

        Args:
            node: The ROS node instance.
        """
        self._node = node
        self._commands: dict[str, Command] = {}

    def load_commands_from_config(self) -> None:
        """
        Load commands from ROS parameters configuration.

        Raises:
            ConfigurationError: If command loading fails.
        """
        try:
            # Get commands from config file
            commands = self._node.get_parameters_by_prefix("commands")

            if not commands:
                self._node.get_logger().warning("No commands found in configuration")
                return

            # Parse command configuration
            commands_dict = self._parse_command_config(commands)

            # Create command objects
            self._create_command_objects(commands_dict)

            self._node.get_logger().info(
                f"Loaded {len(self._commands)} commands from configuration"
            )

        except Exception as e:
            raise ConfigurationError(f"Failed to load commands: {e}") from e

    def _parse_command_config(self, commands: dict[str, Any]) -> dict[str, dict[str, Any]]:
        """
        Parse command configuration parameters.

        Args:
            commands: Raw command parameters.

        Returns:
            Parsed command configuration.
        """
        commands_dict = {}

        for param_name, param in commands.items():
            parts = param_name.split(".")
            if len(parts) != 2:
                continue

            command_num, command_key = parts
            command_value = param.value

            if command_num not in commands_dict:
                commands_dict[command_num] = {}
            commands_dict[command_num][command_key] = command_value

        return commands_dict

    def _create_command_objects(self, commands_dict: dict[str, dict[str, Any]]) -> None:
        """
        Create command objects from configuration.

        Args:
            commands_dict: Parsed command configuration.
        """
        # Available plugins (could be extended)
        plugins = {"CommandPlugin": CommandPlugin}

        for command_config in commands_dict.values():
            try:
                name = command_config.get("name")
                service = command_config.get("service")
                plugin_name = command_config.get("plugin")

                if not all([name, service, plugin_name]):
                    self._node.get_logger().warning(f"Incomplete command config: {command_config}")
                    continue

                if plugin_name not in plugins:
                    self._node.get_logger().error(f"Unknown plugin type: {plugin_name}")
                    continue

                plugin_type = plugins[plugin_name]
                command = Command(self._node, service, plugin_type)
                self._commands[name] = command

                self._node.get_logger().debug(f"Registered command: {name}")

            except Exception as e:
                self._node.get_logger().error(
                    f"Failed to create command from config {command_config}: {e}"
                )

    def get_command(self, name: str) -> Command:
        """
        Get a command by name.

        Args:
            name: Name of the command.

        Returns:
            The command instance.

        Raises:
            CommandNotFoundError: If command is not found.
        """
        if name not in self._commands:
            raise CommandNotFoundError(f"Command '{name}' not found")
        return self._commands[name]

    def has_command(self, name: str) -> bool:
        """
        Check if a command exists.

        Args:
            name: Name of the command.

        Returns:
            True if command exists, False otherwise.
        """
        return name in self._commands

    def list_commands(self) -> list[str]:
        """
        Get list of available command names.

        Returns:
            List of command names.
        """
        return list(self._commands.keys())

    def cleanup(self) -> None:
        """Clean up all commands."""
        for command in self._commands.values():
            try:
                command.cleanup()
            except Exception as e:
                self._node.get_logger().error(f"Error cleaning up command: {e}")
        self._commands.clear()


class CommandExecutorService(CommandExecutor):
    """
    Service for executing commands with proper error handling and result publishing.

    This class implements the CommandExecutor interface and provides a robust
    way to execute commands and handle their results.
    """

    def __init__(self, node: BaseNode, result_publisher: Callable[[Any, str, Any], None]):
        """
        Initialize the command executor.

        Args:
            node: The ROS node instance.
            result_publisher: Function to publish command results.
        """
        self._node = node
        self._result_publisher = result_publisher
        self._registry = CommandRegistry(node)

    def initialize(self) -> None:
        """Initialize the command executor."""
        try:
            self._registry.load_commands_from_config()
            self._node.get_logger().info("Command executor initialized successfully")
        except Exception as e:
            self._node.get_logger().error(f"Failed to initialize command executor: {e}")
            raise

    def execute_command(self, method: str, payload: str, meta: Any) -> Any:
        """
        Execute a command by name.

        Args:
            method: The command method to execute.
            payload: Command payload.
            meta: Command metadata.

        Returns:
            Future representing the command execution.

        Raises:
            CommandNotFoundError: If the command is not found.
            ServiceNotReadyError: If the service is not ready.
        """
        try:
            command = self._registry.get_command(method)
            return command.execute(method, payload, meta, self._service_callback)

        except CommandNotFoundError as e:
            self._node.get_logger().error(str(e))
            self._publish_error(f"Command '{method}' not found", meta)
            raise
        except ServiceNotReadyError as e:
            self._node.get_logger().error(str(e))
            self._publish_error(str(e), meta)
            raise
        except Exception as e:
            self._node.get_logger().error(f"Unexpected error executing command {method}: {e}")
            self._publish_error(f"Command execution failed: {e}", meta)
            raise

    def _service_callback(self, future: Future, payload: str, meta: Any) -> None:
        """
        Callback for handling service call results.

        Args:
            future: The completed future.
            payload: Original command payload.
            meta: Command metadata.
        """
        try:
            result = future.result()
            self._result_publisher(result, payload, meta)
        except Exception as e:
            self._node.get_logger().error(f"Service call failed: {e}")
            self._publish_error(f"Service call failed: {e}", meta)

    def _publish_error(self, error_message: str, meta: Any) -> None:
        """
        Publish an error message.

        Args:
            error_message: The error message.
            meta: Command metadata.
        """
        # Create error result
        error_response = PluginResponse()
        error_response.result_code = 1
        error_response.error_message = error_message
        error_response.error_description = ""

        error_output = CommandOutput()
        error_output.payload = json.dumps({"error": error_message})
        error_output.result = error_response

        self._result_publisher(error_output, "{}", meta)

    def get_available_commands(self) -> list[str]:
        """
        Get list of available commands.

        Returns:
            List of available command names.
        """
        return self._registry.list_commands()

    def cleanup(self) -> None:
        """Clean up the command executor."""
        self._registry.cleanup()
