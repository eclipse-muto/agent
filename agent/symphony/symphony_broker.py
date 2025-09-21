#
# Copyright (c) Microsoft Corporation.
# Licensed under the MIT license.
# SPDX-License-Identifier: MIT
#

"""
Symphony MQTT Broker Module.

This module implements an MQTT broker that handles Symphony COA requests
and responses, providing an interface between Symphony orchestration and
the Muto Agent system.
"""

# Standard library imports
import json
from typing import Any, Dict, Optional

# Third-party imports
from paho.mqtt.client import MQTTMessage

# Local imports
from ..mqtt_manager import MQTTConnectionManager
from .sdk.symphony_sdk import (
    COARequest,
    COAResponse,
    ComparisonPack,
    ComponentSpec,
    DeploymentSpec,
    deserialize_coa_request,
    from_dict,
    serialize_coa_response,
)


class MQTTBroker:
    """
    MQTT Broker for Symphony COA operations.
    
    This class handles MQTT communication for Symphony Component Object
    Architecture (COA) requests and responses, providing an interface
    between Symphony orchestration and the Muto Agent system.
    
    The broker translates HTTP-style requests to MQTT messages and
    vice versa, following the COA protocol specifications.
    """
    
    def __init__(self, plugin, node, config):
        """
        Initialize the MQTT broker.
        
        Args:
            plugin: Symphony provider plugin implementing COA operations.
            node: ROS 2 node for logging and lifecycle management.
            config: Configuration object containing MQTT settings.
        """
        self.init_provider = plugin.init_provider
        self.apply = plugin.apply
        self.remove = plugin.remove
        self.get = plugin.get
        self.needs_update = plugin.needs_update
        self.needs_remove = plugin.needs_remove
        self.logger = node.get_logger()
        self._config = config
        self._node = node
        
        # Initialize MQTT connection manager
        self._mqtt_manager = MQTTConnectionManager(
            node=node,
            config=config.symphony.mqtt,
            message_handler=self._handle_mqtt_message,
            on_connect_handler=self._on_connect,
            logger=self.logger
        )
    
    def _on_connect(self, client, userdata, flags, reason_code, properties):
        """
        Handle MQTT connection established.
        
        Args:
            client: MQTT client instance.
            userdata: User data passed to client.
            flags: Connection flags.
            reason_code: Connection result code.
            properties: Connection properties.
        """
        self.logger.info(
            f"Symphony router connected to MQTT broker with result code "
            f"{reason_code}"
        )

        # Subscribe to Symphony request topic
        request_topic = (
            f"{self._config.symphony.topic_prefix}/"
            f"{self._config.symphony.request_topic}"
        )
        topics = [request_topic]
        
        # Subscribe to request topics
        for topic in topics:
            self._mqtt_manager.subscribe(topic)
            self.logger.debug(f"Subscribed to topic: {topic}")
            
        # Connected to MQTT broker so initialize the provider
        try:
            self.init_provider()
            self.logger.info("Symphony provider initialized successfully")
        except Exception as e:
            self.logger.error(f"Failed to initialize Symphony provider: {e}")
        
    def _handle_mqtt_message(self, message: MQTTMessage) -> None:
        """
        Handle incoming MQTT messages for Symphony operations.
        
        Args:
            message: The MQTT message to handle.
        """
        # Symphony action response topic
        response_topic = (
            f"{self._config.symphony.topic_prefix}/"
            f"{self._config.symphony.response_topic}"
        )

        try:
            topic = message.topic
            payload_str = message.payload.decode('utf-8')
            
            self.logger.info(
                f"Symphony router received MQTT message on topic: {topic}"
            )
            
            # Parse the contents of the payload_str as COARequest
            try:
                coa_request = deserialize_coa_request(payload_str)
            except Exception as e:
                self.logger.error(f"Failed to parse COARequest: {e}")
                error_response = COAResponse.bad_request(
                    f"Invalid COARequest format: {e}"
                )
                error_response.metadata = {}
                self._mqtt_manager.publish(
                    response_topic,
                    serialize_coa_response(error_response)
                )
                return
            
            # Handle request based on method and route in COARequest
            method = coa_request.method
            route = coa_request.route
            body = coa_request.get_body() if coa_request.body else {}
                
            response_data = self._handle_request(
                coa_request.metadata, method, route, body
            )
            
            # Create COAResponse and publish back
            if isinstance(response_data, dict) and "error" in response_data:
                coa_response = COAResponse.error(response_data["error"])
            else:
                coa_response = COAResponse.success(response_data)
            coa_response.metadata = coa_request.metadata
            self._mqtt_manager.publish(
                response_topic, 
                serialize_coa_response(coa_response)
            )
            
        except Exception as e:
            self.logger.error(f"Error handling MQTT message: {e}")
            error_response = COAResponse.error(str(e))
            try:
                error_response.metadata = coa_request.metadata
            except UnboundLocalError:
                error_response.metadata = {}

            self._mqtt_manager.publish(
                response_topic, 
                serialize_coa_response(error_response)
            )
    
    def _handle_request(
        self, 
        metadata: Dict[str, Any], 
        method: str, 
        route: str, 
        body: Dict[str, Any]
    ) -> Any:
        """
        Handle COA request by routing to appropriate handler.
        
        Args:
            metadata: Request metadata.
            method: HTTP method (POST, DELETE, GET).
            route: Request route/endpoint.
            body: Request body data.
            
        Returns:
            Response data from the appropriate handler.
        """
        if route == "instances":
            if method == "POST":
                return self._apply(metadata, body)
            elif method == "DELETE":
                return self._remove(metadata, body)
            elif method == "GET":
                return self._get(metadata, body)
        elif route == "needsupdate":
            return self._needs_update(metadata, body)
        elif route == "needsremove":
            return self._needs_remove(metadata, body)
        else:
            return {"error": "Route not found"}
    
    def _apply(self, metadata: Dict[str, Any], data: Dict[str, Any]) -> str:
        """Apply/deploy components from deployment specification."""
        deployment = from_dict(data, DeploymentSpec)
        components = deployment.get_components_slice()
        return self.apply(metadata, components)
    
    def _remove(self, metadata: Dict[str, Any], data: Dict[str, Any]) -> str:
        """Remove components from deployment specification."""
        deployment = from_dict(data, DeploymentSpec)
        components = deployment.get_components_slice()
        return self.remove(metadata, components)
    
    def _get(self, metadata: Dict[str, Any], data: Dict[str, Any]) -> Any:
        """Get component states from deployment specification."""
        deployment = from_dict(data, DeploymentSpec)
        components = deployment.get_components_slice()
        return self.get(metadata, components)
    
    def _needs_update(
        self, 
        metadata: Dict[str, Any], 
        data: Dict[str, Any]
    ) -> bool:
        """Check if components need updates from comparison pack."""
        pack = from_dict(data, ComparisonPack)
        return self.needs_update(metadata, pack)
    
    def _needs_remove(
        self, 
        metadata: Dict[str, Any], 
        data: Dict[str, Any]
    ) -> bool:
        """Check if components need removal from comparison pack."""
        pack = from_dict(data, ComparisonPack)
        return self.needs_remove(metadata, pack)
    
    def connect(self) -> None:
        """Connect to the MQTT broker."""
        if self._mqtt_manager:
            self._mqtt_manager.connect()
            self.logger.info("Symphony MQTT router connected")

    def stop(self) -> None:
        """Stop the MQTT router."""
        if self._mqtt_manager:
            self._mqtt_manager.disconnect()
            self.logger.info("Symphony MQTT router stopped")

