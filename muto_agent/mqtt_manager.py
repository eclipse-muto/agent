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
MQTT connection management and message handling for the Muto Agent system.
"""

import json
from collections.abc import Callable
from typing import Any

from paho.mqtt.client import CallbackAPIVersion, Client, MQTTMessage, MQTTv5
from paho.mqtt.properties import Properties

from .config import MQTTConfig
from .exceptions import ConnectionError, MessageParsingError
from .interfaces import BaseNode, ConnectionManager, MessageHandler


class MQTTConnectionManager(ConnectionManager):
    """
    Manages MQTT connection with proper error handling and reconnection logic.

    This class provides a robust MQTT connection management system with
    automatic reconnection, proper error handling, and connection monitoring.
    """

    def __init__(
        self,
        node: BaseNode,
        config: MQTTConfig,
        message_handler: Callable[[MQTTMessage], None],
        on_connect_handler: Callable | None = None,
        logger: Any | None = None,
    ):
        """
        Initialize the MQTT connection manager.

        Args:
            config: MQTT configuration.
            message_handler: Function to handle incoming MQTT messages.
            logger: Optional logger instance.
        """
        self._config = config
        self._message_handler = message_handler
        self._on_connect_handler = on_connect_handler
        self._client: Client | None = None
        self._connected = False
        self._node = node
        self.get_logger = node.get_logger

        self._initialize_client()

    def _log(self, level: str, message: str) -> None:
        """Log message if logger is available."""
        if self.get_logger():
            getattr(self.get_logger(), level)(message)

    def _initialize_client(self) -> None:
        """Initialize the MQTT client with proper configuration."""
        try:
            client_id = f"{self._config.name}_{hash(self._config.namespace)}"

            self._client = Client(
                callback_api_version=CallbackAPIVersion.VERSION2,
                client_id=client_id,
                reconnect_on_failure=True,
                protocol=MQTTv5,
            )

            # Set callbacks
            self._client.on_connect = self._on_connect
            self._client.on_message = self._on_message
            self._client.on_disconnect = self._on_disconnect

            # Set credentials if provided
            if self._config.user and self._config.password:
                self._client.username_pw_set(self._config.user, self._config.password)

            self.get_logger().debug("MQTT client initialized")

        except Exception as e:
            raise ConnectionError(f"Failed to initialize MQTT client: {e}") from e

    def connect(self) -> bool:
        """
        Establish MQTT connection.

        Returns:
            True if connection successful, False otherwise.

        Raises:
            ConnectionError: If connection fails.
        """
        if not self._client:
            raise ConnectionError("MQTT client not initialized")

        try:
            result_code = self._client.connect(
                self._config.host, self._config.port, self._config.keep_alive
            )

            if result_code == 0:
                self._client.loop_start()
                self.get_logger().info(
                    f"MQTT connection established to {self._config.host}:{self._config.port}"
                )
                return True
            else:
                raise ConnectionError(f"MQTT connection failed with code: {result_code}")

        except Exception as e:
            self.get_logger().error(f"Failed to connect to MQTT broker: {e}")
            raise ConnectionError(f"MQTT connection failed: {e}") from e

    def disconnect(self) -> None:
        """Disconnect from MQTT broker."""
        if self._client:
            try:
                self._client.loop_stop()
                self._client.disconnect()
                self._connected = False
                self.get_logger().info("MQTT disconnected")
            except Exception as e:
                self.get_logger().error(f"Error during MQTT disconnect: {e}")

    def is_connected(self) -> bool:
        """
        Check if currently connected to MQTT broker.

        Returns:
            True if connected, False otherwise.
        """
        return self._connected and self._client is not None and self._client.is_connected()

    def publish(self, topic: str, payload: str, properties: Properties | None = None) -> bool:
        """
        Publish a message to MQTT broker.

        Args:
            topic: The topic to publish to.
            payload: The message payload.
            properties: Optional MQTT properties.

        Returns:
            True if publish successful, False otherwise.
        """
        if not self.is_connected():
            self.get_logger().error("Cannot publish: MQTT not connected")
            return False

        try:
            result = self._client.publish(topic, payload, properties=properties)
            if result.rc == 0:
                self.get_logger().debug(f"Message published to topic: {topic}")
                return True
            else:
                self.get_logger().error(f"Failed to publish message, return code: {result.rc}")
                return False

        except Exception as e:
            self.get_logger().error(f"Error publishing message: {e}")
            return False

    def subscribe(self, topic: str) -> bool:
        """
        Subscribe to an MQTT topic.

        Args:
            topic: The topic to subscribe to.

        Returns:
            True if subscription successful, False otherwise.
        """
        if not self._client:
            self.get_logger().error("Cannot subscribe: MQTT client not initialized")
            return False

        try:
            result, _ = self._client.subscribe(topic)
            if result == 0:
                self.get_logger().info(f"Subscribed to topic: {topic}")
                return True
            else:
                self.get_logger().error(
                    f"Failed to subscribe to topic {topic}, return code: {result}"
                )
                return False

        except Exception as e:
            self.get_logger().error(f"Error subscribing to topic {topic}: {e}")
            return False

    def _on_connect(self, client, userdata, flags, reason_code, properties) -> None:
        """
        Callback for MQTT connection events.

        Args:
            client: The client instance.
            userdata: Private user data.
            flags: Response flags.
            reason_code: Connection result code.
            properties: MQTT v5 properties.
        """

        if reason_code == 0:
            self._connected = True

            # If there is a self.on_connect_handler use it otherwise default
            if self._on_connect_handler is not None:
                self._on_connect_handler(client, userdata, flags, reason_code, properties)
            else:
                # default behavior
                # Subscribe to twin topic
                twin_topic = f"{self._config.prefix}/{self._config.namespace}:{self._config.name}"
                self.subscribe(twin_topic)
                self.get_logger().info(f"MQTT connected and subscribed to {twin_topic}")
        else:
            self._connected = False
            self.get_logger().error(f"MQTT connection failed with reason code: {reason_code}")

    def _on_disconnect(self, client, userdata, flags, reason_code, properties) -> None:
        """
        Callback for MQTT disconnection events.

        Args:
            client: The client instance.
            userdata: Private user data.
            flags: Disconnect flags.
            reason_code: Disconnection reason code.
            properties: MQTT v5 properties.
        """
        self._connected = False
        self.get_logger().warning(f"MQTT disconnected with reason code: {reason_code}")

    def _on_message(self, client, userdata, message: MQTTMessage) -> None:
        """
        Callback for incoming MQTT messages.

        Args:
            client: The client instance.
            userdata: Private user data.
            message: The received message.
        """
        try:
            if self._message_handler:
                self._message_handler(message)
        except Exception as e:
            self.get_logger().error(f"Error handling MQTT message: {e}")


class DittoMessageHandler(MessageHandler):
    """
    Handler for Ditto protocol messages received via MQTT.

    This class processes incoming Ditto protocol messages and converts
    them into appropriate internal message formats.
    """

    def __init__(
        self,
        namespace: str,
        name: str,
        agent_publisher: Callable,
        thing_publisher: Callable,
        error_publisher: Callable,
        logger: Any | None = None,
    ):
        """
        Initialize the Ditto message handler.

        Args:
            namespace: The namespace for the device.
            name: The name of the device.
            agent_publisher: Function to publish messages to agent.
            thing_publisher: Function to publish thing messages.
            error_publisher: Function to publish error messages.
        """
        self._namespace = namespace
        self._name = name
        self._agent_publisher = agent_publisher
        self._thing_publisher = thing_publisher
        self._error_publisher = error_publisher

        # Topic pattern for parsing things messages
        self._things_pattern = r".*/things/([^/]*)/([^/]*)/(.*)"
        self._logger = logger

    def handle_message(self, message: MQTTMessage) -> None:
        """
        Handle an incoming MQTT message.

        Args:
            message: The MQTT message to handle.

        Raises:
            MessageParsingError: If message parsing fails.
        """
        try:
            # Parse basic message properties
            topic = message.topic
            payload = message.payload.decode("utf-8")
            properties = message.properties

            # Parse JSON payload
            thing_message = json.loads(payload)

            # Extract message components
            thing_topic = thing_message.get("topic", "")
            headers = thing_message.get("headers", {})
            path = thing_message.get("path", "")
            value = thing_message.get("value", "")

            # Create metadata
            meta = self._create_meta_from_headers(headers)

            # Process based on message type
            if "things/twin/errors" in thing_topic:
                self._handle_error_message(thing_message)
            else:
                self._handle_things_message(thing_message, meta)

        except json.JSONDecodeError as e:
            self._logger.error(f"Failed to parse JSON payload: {e}")
            raise MessageParsingError(f"Invalid JSON in message: {e}") from e
        except Exception as e:
            self._logger.error(f"Failed to handle MQTT message: {e}")
            raise MessageParsingError(f"Message handling failed: {e}") from e

    def _create_meta_from_headers(self, headers: dict[str, Any]) -> Any:
        """
        Create metadata object from message headers.

        Args:
            headers: Message headers dictionary.

        Returns:
            Metadata object for ROS message.
        """
        # Import here to avoid circular imports
        from muto_msgs.msg import MutoActionMeta

        meta = MutoActionMeta()
        meta.response_topic = headers.get("reply-to", "")
        meta.correlation_data = headers.get("correlation-id", "")

        return meta

    def _handle_error_message(self, thing_message: dict[str, Any]) -> None:
        """
        Handle error messages from Ditto.

        Args:
            thing_message: The parsed thing message.
        """
        self._logger.warning("Ditto error message received")
        # Could be extended to handle specific error processing

    def _handle_things_message(self, thing_message: dict[str, Any], meta: Any) -> None:
        """
        Handle regular things protocol messages.

        Args:
            thing_message: The parsed thing message.
            meta: Message metadata.
        """
        try:
            import re

            thing_topic = thing_message.get("topic", "")
            path = thing_message.get("path", "")

            # Parse things topic
            parsed = re.findall(self._things_pattern, thing_topic)

            if parsed and len(parsed[0]) >= 3:
                channel = parsed[0][0]
                criterion = parsed[0][1]
                action_parts = parsed[0][2].split("/")

                # Check if this is a live message for agent/stack
                if (
                    channel == "live"
                    and criterion == "messages"
                    and len(action_parts) > 0
                    and action_parts[0] in ["agent", "stack"]
                ):
                    if path.startswith("/inbox"):
                        self._agent_publisher(thing_message, meta)
                    else:
                        self._thing_publisher(thing_message, channel, action_parts[0], meta)
                else:
                    self._thing_publisher(
                        thing_message, channel, action_parts[0] if action_parts else "", meta
                    )
            else:
                self._error_publisher(
                    meta,
                    status=400,
                    error="things:topic.malformed",
                    message="Ditto Protocol message topic is malformed.",
                )

        except Exception as e:
            self._logger.error(f"Error processing things message: {e}")
            self._error_publisher(
                meta,
                status=400,
                error="things:ditto.unsupported",
                message="Message is not a supported Ditto Protocol message.",
            )
