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

"""
Muto Symphony Provider.

This module implements a Symphony provider that integrates with the Muto Agent
MQTT infrastructure, allowing Symphony orchestrators to manage ROS 2 components
through the Muto platform.
"""

# Standard library imports
import base64
import json
import signal
import threading
from typing import Any, Dict, List, Optional

# Third-party imports
import rclpy
from muto_msgs.msg import MutoAction
from rclpy.node import Node
from std_srvs.srv import Trigger

# Local imports
from ..config import ConfigurationManager
from ..interfaces import BaseNode
from .sdk.symphony_api import SymphonyAPIClient, SymphonyAPIError
from .sdk.symphony_sdk import (
    ComponentSpec,
    ComparisonPack,
    SymphonyProvider,
    to_dict,
)
from .sdk.symphony_summary import (
    ComponentResultSpec,
    SummarySpec,
    SummaryState,
    TargetResultSpec,
)
from .sdk.symphony_types import State
from .symphony_broker import MQTTBroker


class MutoSymphonyProvider(BaseNode, SymphonyProvider):
    """
    Symphony provider integrating with Muto Agent MQTT infrastructure.
    
    This provider implements the Symphony provider interface and manages
    ROS 2 components through Muto's existing MQTT communication layer.
    """
    
    def __init__(
        self, 
        config_manager: Optional[ConfigurationManager] = None, 
        node_name: str = 'muto_symphony_provider'
    ):
        """
        Initialize the Muto Symphony Provider.
        
        Args:
            config_manager: Optional configuration manager.
            node_name: ROS 2 node name.
        """
        super().__init__(node_name)
        
        # Configuration
        self._config_manager = config_manager or ConfigurationManager(self)
        self._config = self._config_manager.load_config()
        
        # MQTT Broker for Symphony communication
        self._mqtt_broker: Optional[MQTTBroker] = None
        
        # Symphony API client
        self._api_client: Optional[SymphonyAPIClient] = None
        
        # Lifecycle management
        self._shutdown_event = threading.Event()
        self._running = False
        
        # Setup logging
        self.logger = self.get_logger()
        
        # Setup stack publisher
        topics = self._config.topics
        self.stack_publisher = self.create_publisher(
            MutoAction, topics.stack_topic, 10
        )
    

    def _do_initialize(self) -> None:
    
        # Create ROS services for Symphony registration
        self._register_service = self.create_service(
            Trigger,
            'symphony_register_target',
            self._register_target_service
        )
        
        self._unregister_service = self.create_service(
            Trigger,
            'symphony_unregister_target', 
            self._unregister_target_service
        )
        
        # Initialize Symphony API client
        self._init_symphony_api()
        
        # Initialize MQTT Broker if enabled
        if self._config.symphony.enabled:
            self._init_mqtt_broker()
            
    
    def _init_symphony_api(self) -> None:
        """Initialize Symphony API client."""
        try:
            symphony = self._config.symphony
            self._api_client = SymphonyAPIClient(
                base_url=symphony.api_url,
                username=symphony.mqtt.user,
                password=symphony.mqtt.password,
                timeout=30.0,
                logger=self.logger
            )
            
            self.logger.info("Symphony API client initialized")
            
        except Exception as e:
            self.logger.error(f"Failed to initialize Symphony API client: {e}")
            self._api_client = None

    def _init_mqtt_broker(self) -> None:
        """Initialize MQTT broker for Symphony operations."""
        try:
            # Initialize the MQTTBroker with this provider as the plugin
            self._mqtt_broker = MQTTBroker(
                plugin=self,  # This provider implements the required methods
                node=self,    # ROS node
                config=self._config  # Configuration
            )
            self._mqtt_broker.connect()
             
        except Exception as e:
            self.logger.error(f"Failed to initialize MQTT Broker: {e}")
            self._mqtt_broker = None





    def _authenticate_symphony_api(self) -> Optional[str]:
        """
        Authenticate with Symphony API and return access token.
        
        Returns:
            Access token if successful, None if failed.
        """
        if not self._api_client:
            self.logger.error("Symphony API client not initialized")
            return None
            
        try:
            return self._api_client.authenticate()
        except SymphonyAPIError as e:
            self.logger.error(f"Symphony API authentication failed: {e}")
            return None

    def register_target(self) -> bool:
        """
        Register this Muto agent as a target in Symphony.
        
        Returns:
            True if registration successful, False otherwise.
        """
        if not self._api_client:
            self.logger.error("Symphony API client not initialized")
            return False
            
        symphony = self._config.symphony
        try:
            # Build target registration payload
            target_payload = {
                "metadata": {
                    "name": symphony.target
                },
                "spec": {
                    "displayName": symphony.target,
                    "forceRedeploy": True,
                    "topologies": [
                        {
                            "bindings": [
                                {
                                    "role": "muto-agent",
                                    "provider": symphony.provider_name,
                                    "config": {
                                        "name": "proxy",
                                        "brokerAddress": symphony.broker_address,
                                        "clientID": symphony.client_id,
                                        "requestTopic": f"{symphony.topic_prefix}/{symphony.request_topic}",
                                        "responseTopic": f"{symphony.topic_prefix}/{symphony.response_topic}",
                                        "timeoutSeconds": symphony.timeout_seconds
                                    }
                                }
                            ]
                        }
                    ]
                }
            }
            
            # Register target using API client
            self._api_client.register_target(symphony.target, target_payload)
            return True
            
        except SymphonyAPIError as e:
            self.logger.error(f"Failed to register target with Symphony: {e}")
            return False
        except Exception as e:
            self.logger.error(f"Unexpected error registering target: {e}")
            return False

    def unregister_target(self) -> bool:
        """
        Unregister this Muto agent target from Symphony.
        
        Returns:
            True if unregistration successful, False otherwise.
        """
        if not self._api_client:
            self.logger.error("Symphony API client not initialized")
            return False
            
        symphony = self._config.symphony
        try:
            # Unregister target using API client
            self._api_client.unregister_target(symphony.target, direct=True)
            return True
            
        except SymphonyAPIError as e:
            self.logger.error(f"Failed to unregister target from Symphony: {e}")
            return False
        except Exception as e:
            self.logger.error(f"Unexpected error unregistering target: {e}")
            return False

    def _auto_register_target(self) -> None:
        """
        Automatically register target with Symphony if conditions are met.
        This is called after MQTT broker initialization.
        """
        try:
            if self._mqtt_broker:
                self.logger.info("Attempting auto-registration with Symphony")
                success = self.register_target()
                if success:
                    self.logger.info("Auto-registration with Symphony completed successfully")
                else:
                    self.logger.warn("Auto-registration with Symphony failed, will not retry automatically")
            else:
                self.logger.info("MQTT broker not available for auto-registration")
        except Exception as e:
            self.logger.error(f"Error in auto-registration: {e}")

    def _register_target_service(
        self, 
        request: Trigger.Request, 
        response: Trigger.Response
    ) -> Trigger.Response:
        """
        ROS service callback to register target with Symphony.
        
        Args:
            request: Service request.
            response: Service response.
            
        Returns:
            Service response with success status and message.
        """
        success = self.register_target()
        response.success = success
        if success:
            response.message = f"Successfully registered target '{self._config.symphony.target}' with Symphony"
        else:
            response.message = f"Failed to register target '{self._config.symphony.target}' with Symphony"
        return response

    def _unregister_target_service(
        self, 
        request: Trigger.Request, 
        response: Trigger.Response
    ) -> Trigger.Response:
        """
        ROS service callback to unregister target from Symphony.
        
        Args:
            request: Service request.
            response: Service response.
            
        Returns:
            Service response with success status and message
        """
        success = self.unregister_target()
        response.success = success
        if success:
            response.message = f"Successfully unregistered target '{self._config.symphony.target}' from Symphony"
        else:
            response.message = f"Failed to unregister target '{self._config.symphony.target}' from Symphony"
        return response

    # SymphonyProvider Interface Methods
    # These are called by MQTTBroker when it receives Symphony requests

    def init_provider(self):
        """Initialize the provider - required by MQTTBroker interface."""
        self.logger.info(f"Muto Symphony Provider initialized - Target: {self._config.symphony.target}")    
        # Auto-register target after initialization
        self.register_target()
        
    def apply(
        self, 
        metadata: Dict[str, Any], 
        components: List[ComponentSpec]
    ) -> str:
        """
        Apply/deploy components.
        
        Called by MQTTBroker when Symphony sends deployment requests.
        This method implements the core deployment logic for Symphony 
        components.
        
        Args:
            metadata: Request metadata.
            components: List of components to deploy.
            
        Returns:
            JSON string containing deployment results.
        """
        self.logger.info(
            f"Symphony apply: deploying {len(components)} components"
        )

        # Use summary models
        result = SummarySpec(target_count=1, success_count=1)
        target_result = TargetResultSpec()
        
        for component in components:
            self.logger.info(
                f"Deploying component: {component.name} of type "
                f"{component.type}"
            )
            
            component_type = component.properties.get("type", "")
            content_type = component.properties.get("content-type", "")
            data = component.properties.get("data", "")
            
            if (component_type == "stack" and 
                content_type == "application/json" and 
                data):
                try:
                    stack_json_str = base64.b64decode(data).decode('utf-8')
                    stack_data = json.loads(stack_json_str)
                    self.logger.info(
                        f"Decoded stack data for {component_type} "
                        f"{component.name}: {stack_data}"
                    )
                    
                    msg_action = MutoAction()
                    msg_action.context = ""
                    msg_action.method = "start"
                    msg_action.payload = json.dumps(stack_data)
                    self.stack_publisher.publish(msg_action)

                    # Implement actual deployment logic using stack_data
                except Exception as e:
                    self.logger.error(
                        f"Failed to decode or parse stack data for "
                        f"{component_type} {component.name}: {e}"
                    )
                    
            component_result = ComponentResultSpec()
            component_result.status = State.UPDATED
            component_result.message = (
                f"Deploying component: {component.name} of type "
                f"{component.type}"
            )
            
            target_result.component_results[component.name] = component_result
            self.logger.info(
                f"Deploying component: {component.name} of type "
                f"{component.type}"
            )
        
        target_result.state = SummaryState.DONE
        result.update_target_result(metadata["active-target"], target_result)
        
        return json.dumps(result.to_dict(), indent=2)

    def remove(
        self, 
        metadata: Dict[str, Any], 
        components: List[ComponentSpec]
    ) -> str:
        """
        Remove components.
        
        Called by MQTTBroker when Symphony sends removal requests.
        This method implements the core removal logic for Symphony components.
        
        Args:
            metadata: Request metadata.
            components: List of components to remove.
            
        Returns:
            JSON string containing removal results.
        """
        self.logger.info(
            f"Symphony remove: removing {len(components)} components"
        )
        
        results = []
        
        return json.dumps([to_dict(result) for result in results], indent=2)

    def get(
        self, 
        metadata: Dict[str, Any], 
        components: List[ComponentSpec]
    ) -> Any:
        """
        Get current component states.
        
        Called by MQTTBroker when Symphony requests component info.
        This method returns the current state of deployed components.
        
        Args:
            metadata: Request metadata.
            components: List of components to query.
            
        Returns:
            List of component data dictionaries.
        """
        self.logger.info(
            f"Symphony get: retrieving state for {len(components)} components"
        )
        
        # Return the requested components or empty list if none specified
        # In a real implementation, this would query the actual system state
        if not components:
            current_components = []
            self.logger.info(
                "No specific components requested, returning empty list"
            )
        else:
            # Return the requested components as-is for now
            # In real implementation, query actual deployment status
            current_components = components
            self.logger.info(
                f"Returning {len(current_components)} requested components"
            )
        
        return [to_dict(comp) for comp in current_components]

    def needs_update(
        self, 
        metadata: Dict[str, Any], 
        pack: ComparisonPack
    ) -> bool:
        """
        Check if components need updates by comparing desired vs current state.
        
        Args:
            metadata: Request metadata.
            pack: Comparison pack containing desired and current components.
            
        Returns:
            True if updates are needed, False otherwise.
        """
        self.logger.info(
            f"Checking update need for {len(pack.desired)} desired vs "
            f"{len(pack.current)} current"
        )
        
        # Create lookup for current components
        current_by_name = {comp.name: comp for comp in pack.current}
        
        for desired in pack.desired:
            current = current_by_name.get(desired.name)
            
            if not current:
                # Component doesn't exist, needs deployment
                self.logger.info(
                    f"Component {desired.name} not found - needs deployment"
                )
                return True
            
            # Check if component properties have changed
            if self._component_changed(desired, current):
                self.logger.info(f"Component {desired.name} has changes - needs update")
                return True
        
        # No updates needed
        self.logger.info("No components need updates")
        return False

    def needs_remove(self, metadata: Dict[str, Any], pack: ComparisonPack) -> bool:
        """Check if components need removal by comparing desired vs current state."""
        self.logger.info(f"Checking removal need for {len(pack.desired)} desired vs {len(pack.current)} current")
        
        # Create lookup for desired components
        desired_by_name = {comp.name: comp for comp in pack.desired}
        
        for current in pack.current:
            if current.name not in desired_by_name:
                # Current component not in desired state, needs removal
                self.logger.info(f"Component {current.name} not desired - needs removal")
                return True
        
        # No removals needed
        self.logger.info("No components need removal")
        return False

    def _component_changed(self, desired: ComponentSpec, current: ComponentSpec) -> bool:
        """Check if a component has changed between desired and current state."""
        # Compare key properties that would require updates
        
        # Type change
        if desired.type != current.type:
            return True
        
        # Properties change
        if desired.properties != current.properties:
            return True
        
        # Parameters change
        if desired.parameters != current.parameters:
            return True
        
        # Constraints change
        if desired.constraints != current.constraints:
            return True
        
        # Dependencies change
        if desired.dependencies != current.dependencies:
            return True
        
        return False

    def start(self) -> bool:
        """Start the Symphony provider."""
        if not self._config.symphony.enabled:
            self.logger.info("Symphony provider not enabled in configuration")
            return False
        
        if self._running:
            self.logger.warning("Symphony provider already running")
            return True
        
        try:
            # The MQTT broker is initialized in __init__ if enabled
            # No additional startup steps needed
            self._running = True
            self.logger.info("Muto Symphony Provider started successfully")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to start Symphony provider: {e}")
            return False

    def _do_cleanup(self) -> None:
        """Stop the Symphony provider."""
        if not self._running:
            return  # Already cleaned up
            
        self._running = False
        self.logger.info("Stopping Muto Symphony Provider")
        
        # Set shutdown event first to signal other threads
        self._shutdown_event.set()
        
        try:
            # Unregister from Symphony
            self.unregister_target()
        except Exception as e:
            self.logger.error(f"Error during Symphony unregistration: {e}")
        
        try:
            # Stop MQTT broker
            if self._mqtt_broker:
                self._mqtt_broker.stop()
                self._mqtt_broker = None
        except Exception as e:
            self.logger.error(f"Error stopping MQTT broker: {e}")
        
        try:
            # Close API client
            if self._api_client:
                self._api_client.close()
                self._api_client = None
        except Exception as e:
            self.logger.error(f"Error closing API client: {e}")
        
        try:
            # Destroy ROS2 services and publishers
            if hasattr(self, '_register_service') and self._register_service:
                self.destroy_service(self._register_service)
            if hasattr(self, '_unregister_service') and self._unregister_service:
                self.destroy_service(self._unregister_service)
            if hasattr(self, 'stack_publisher') and self.stack_publisher:
                self.destroy_publisher(self.stack_publisher)
        except Exception as e:
            self.logger.error(f"Error destroying ROS2 resources: {e}")
        
        self.logger.info("Muto Symphony Provider stopped")

    def is_running(self) -> bool:
        """Check if the provider is running."""
        return self._running and not self._shutdown_event.is_set()

    def get_target_name(self) -> str:
        """Get the Symphony target name."""
        return self._config.symphony.target

    def get_component_count(self) -> int:
        """Get the number of managed components."""
        # Component tracking removed - return 0 for now
        # In a real implementation, this would query the actual system
        return 0




def main():
    """Main entry point for the Symphony Provider."""
    provider = None
    shutdown_requested = threading.Event()
    
    def signal_handler(signum, frame):
        """Handle shutdown signals gracefully."""
        print(f"Received signal {signum}, initiating graceful shutdown...")
        shutdown_requested.set()
        if provider is not None:
            provider._shutdown_event.set()
    
    # Register signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        rclpy.init()
        provider = MutoSymphonyProvider()
        provider.initialize()
        
        provider.get_logger().info("Symphony Provider started successfully")
        
        # Custom spin loop to handle shutdown gracefully
        while rclpy.ok() and not shutdown_requested.is_set():
            try:
                rclpy.spin_once(provider, timeout_sec=1.0)
            except KeyboardInterrupt:
                break
        
    except KeyboardInterrupt:
        print("Symphony Provider interrupted by user")
    except Exception as e:
        print(f"Failed to start Symphony Provider: {e}")
        
    finally:
        # Cleanup provider if it was created
        if provider is not None:
            try:
                print("Cleaning up Symphony Provider...")
                provider.cleanup()
            except Exception as e:
                print(f"Error during provider cleanup: {e}")
        
        # Only shutdown ROS2 if it's still initialized and we haven't already shut it down
        try:
            if rclpy.ok():
                print("Shutting down ROS2...")
                rclpy.shutdown()
        except Exception as e:
            print(f"Error during ROS2 shutdown (this may be normal): {e}")

if __name__ == '__main__':
    exit(main())