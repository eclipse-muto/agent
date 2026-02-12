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
from typing import Any

# Third-party imports
import rclpy
from muto_msgs.msg import MutoAction
from std_srvs.srv import Trigger
from symphony_sdk import (
    ComparisonPack,
    ComponentResultSpec,
    ComponentSpec,
    State,
    SummarySpec,
    SummaryState,
    SymphonyAPI,
    SymphonyAPIError,
    TargetResultSpec,
    to_dict,
)

# Local imports
from ..config import ConfigurationManager
from ..interfaces import BaseNode
from .provider_base import SymphonyProvider
from .symphony_broker import MQTTBroker


class MutoSymphonyProvider(BaseNode, SymphonyProvider):
    """
    Symphony provider integrating with Muto Agent MQTT infrastructure.

    This provider implements the Symphony provider interface and manages
    ROS 2 components through Muto's existing MQTT communication layer.
    """

    def __init__(
        self,
        config_manager: ConfigurationManager | None = None,
        node_name: str = "muto_symphony_provider",
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
        self._mqtt_broker: MQTTBroker | None = None

        # Symphony API client
        self._api_client: SymphonyAPI | None = None

        # Lifecycle management
        self._shutdown_event = threading.Event()
        self._running = False

        # Setup logging
        self.logger = self.get_logger()

        # Setup stack publisher
        topics = self._config.topics
        self.stack_publisher = self.create_publisher(MutoAction, topics.stack_topic, 10)

        self._component_registry: dict[str, dict[str, Any]] = {}

    def _do_initialize(self) -> None:

        # Create ROS services for Symphony registration
        self._register_service = self.create_service(
            Trigger, "symphony_register_target", self._register_target_service
        )

        self._unregister_service = self.create_service(
            Trigger, "symphony_unregister_target", self._unregister_target_service
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
            self._api_client = SymphonyAPI(
                base_url=symphony.api_url,
                username=symphony.mqtt.user,
                password=symphony.mqtt.password,
                timeout=30.0,
                logger=self.logger,
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
                node=self,  # ROS node
                config=self._config,  # Configuration
            )
            self._mqtt_broker.connect()

        except Exception as e:
            self.logger.error(f"Failed to initialize MQTT Broker: {e}")
            self._mqtt_broker = None

    def _authenticate_symphony_api(self) -> str | None:
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
                                    "timeoutSeconds": symphony.timeout_seconds,
                                },
                            }
                        ]
                    }
                ],
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
                    self.logger.warn(
                        "Auto-registration with Symphony failed, will not retry automatically"
                    )
            else:
                self.logger.info("MQTT broker not available for auto-registration")
        except Exception as e:
            self.logger.error(f"Error in auto-registration: {e}")

    def _register_target_service(
        self, request: Trigger.Request, response: Trigger.Response
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
            response.message = (
                f"Successfully registered target '{self._config.symphony.target}' with Symphony"
            )
        else:
            response.message = (
                f"Failed to register target '{self._config.symphony.target}' with Symphony"
            )
        return response

    def _unregister_target_service(
        self, request: Trigger.Request, response: Trigger.Response
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
            response.message = (
                f"Successfully unregistered target '{self._config.symphony.target}' from Symphony"
            )
        else:
            response.message = (
                f"Failed to unregister target '{self._config.symphony.target}' from Symphony"
            )
        return response

    # SymphonyProvider Interface Methods
    # These are called by MQTTBroker when it receives Symphony requests

    def init_provider(self):
        """Initialize the provider - required by MQTTBroker interface."""
        self.logger.info(
            f"Muto Symphony Provider initialized - Target: {self._config.symphony.target}"
        )
        # Auto-register target after initialization
        self.register_target()

    def apply(self, metadata: dict[str, Any], components: list[ComponentSpec]) -> str:
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
        self.logger.info(f"Symphony apply: deploying {len(components)} components")

        # Use summary models
        result = SummarySpec(target_count=1)
        target_result = TargetResultSpec()
        successes = 0
        failures = 0

        target_name = metadata.get("active-target", self.get_target_name())

        for component in components:
            component_name = component.name or "unnamed-component"
            self.logger.info(f"Deploying component: {component_name} of type {component.type}")

            component_result = ComponentResultSpec()

            stack_payload, decode_error = self._extract_stack_payload(component)
            if decode_error:
                failures += 1
                component_result.status = State.UPDATE_FAILED
                component_result.message = decode_error
                target_result.component_results[component_name] = component_result
                self.logger.error(f"Component {component_name} payload error: {decode_error}")
                continue

            publish_method = self._resolve_component_method(component, default="apply")
            published = self._publish_stack_action(
                method=publish_method,
                payload=stack_payload,
                context=component.metadata.get("context", "") if component.metadata else "",
            )

            if not published:
                failures += 1
                component_result.status = State.MQTT_PUBLISH_FAILED
                component_result.message = (
                    f"Failed to publish apply action for component {component_name}"
                )
                target_result.component_results[component_name] = component_result
                self.logger.error(component_result.message)
                # Do NOT update _component_registry on failed publish
                continue

            # Only update registry after confirmed successful publish
            successes += 1
            component_result.status = State.UPDATED
            component_result.message = f"Apply action published for component {component_name}"
            target_result.component_results[component_name] = component_result

            # Registry mutation: only performed after successful stack action publish
            self._component_registry[component_name] = {
                "component": to_dict(component),
                "payload": stack_payload,
                "status": "applied",
                "state": State.UPDATED.value,
                "last_action": publish_method,
            }

        target_result.status = "OK" if failures == 0 else "FAILED"
        if failures:
            target_result.message = f"{failures} component(s) failed during apply"

        result.success_count = successes
        result.current_deployed = successes
        result.planned_deployment = len(components)
        if failures:
            result.summary_message = target_result.message

        target_result.state = SummaryState.DONE
        result.update_target_result(target_name, target_result)

        return json.dumps(result.to_dict(), indent=2)

    def remove(self, metadata: dict[str, Any], components: list[ComponentSpec]) -> str:
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
        self.logger.info(f"Symphony remove: removing {len(components)} components")

        result = SummarySpec(target_count=1, is_removal=True)
        target_result = TargetResultSpec()
        successes = 0
        failures = 0
        target_name = metadata.get("active-target", self.get_target_name())

        for component in components:
            component_name = component.name or "unnamed-component"
            self.logger.info(f"Removing component: {component_name} of type {component.type}")

            component_result = ComponentResultSpec()

            stack_payload, decode_error = self._extract_stack_payload(
                component, allow_registry_lookup=True
            )

            if decode_error:
                failures += 1
                component_result.status = State.DELETE_FAILED
                component_result.message = decode_error
                target_result.component_results[component_name] = component_result
                self.logger.error(f"Component {component_name} payload error: {decode_error}")
                continue

            publish_method = self._resolve_component_method(component, default="kill")
            published = self._publish_stack_action(
                method=publish_method,
                payload=stack_payload,
                context=component.metadata.get("context", "") if component.metadata else "",
            )

            if not published:
                failures += 1
                component_result.status = State.DELETE_FAILED
                component_result.message = (
                    f"Failed to publish remove action for component {component_name}"
                )
                target_result.component_results[component_name] = component_result
                self.logger.error(component_result.message)
                # Do NOT modify _component_registry on failed publish
                continue

            # Only update registry after confirmed successful publish
            successes += 1
            component_result.status = State.DELETED
            component_result.message = f"Remove action published for component {component_name}"
            target_result.component_results[component_name] = component_result

            # Registry mutation: only performed after successful stack action publish
            self._component_registry.pop(component_name, None)

        target_result.status = "OK" if failures == 0 else "FAILED"
        if failures:
            target_result.message = f"{failures} component(s) failed during removal"

        result.success_count = successes
        if successes:
            result.removed = True
        if failures:
            result.summary_message = target_result.message

        target_result.state = SummaryState.DONE
        result.update_target_result(target_name, target_result)

        return json.dumps(result.to_dict(), indent=2)

    def get(self, metadata: dict[str, Any], components: list[ComponentSpec]) -> Any:
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
        self.logger.info(f"Symphony get: retrieving state for {len(components)} components")

        # Determine which components to include in the response
        if not components:
            component_names = list(self._component_registry.keys())
            self.logger.info("No specific components requested, returning tracked components")
        else:
            component_names = [comp.name for comp in components if comp.name]

        # Read-only operation: get() does NOT modify _component_registry
        # Only report components that are successfully found in the registry
        reported_components: list[dict[str, Any]] = []
        for component_name in component_names:
            state_entry = self._component_registry.get(component_name)
            if not state_entry:
                # Component not found - do not create or persist any state
                self.logger.debug(f"Component {component_name} not found in registry; skipping")
                continue

            # Successfully retrieved component state from registry
            # Create a copy to avoid external mutation of registry data
            try:
                component_info = dict(state_entry["component"])
                component_info["status"] = state_entry.get("status", "unknown")
                component_info["state"] = state_entry.get("state", State.NONE.value)
                component_info["last_action"] = state_entry.get("last_action", "")
                reported_components.append(component_info)
            except (KeyError, TypeError) as exc:
                # Failed to retrieve component state - do not persist stale data
                self.logger.warning(
                    f"Failed to retrieve state for component {component_name}: {exc}"
                )
                continue

        return json.dumps(reported_components, indent=2)

    def _resolve_component_method(self, component: ComponentSpec, default: str) -> str:
        """Determine the composer method for the provided component."""
        props = component.properties or {}
        method = props.get("method") or props.get("action")

        if not method and component.parameters:
            method = component.parameters.get("method") or component.parameters.get("action")

        if not method and component.metadata:
            method = component.metadata.get("method") or component.metadata.get("action")

        resolved = (method or default).lower()

        if default == "kill" and resolved == "start":
            # Prevent accidental start when the default is a destructive action
            return default

        return resolved

    def _extract_stack_payload(
        self, component: ComponentSpec, allow_registry_lookup: bool = False
    ) -> tuple[Any | None, str | None]:
        """Extract a JSON payload representing the stack for a component."""
        try:
            props = component.properties or {}
            data = props.get("data")

            if data is None and allow_registry_lookup:
                registry_entry = self._component_registry.get(component.name or "")
                if registry_entry:
                    return registry_entry.get("payload"), None
                return None, "Component stack payload not available"

            if isinstance(data, dict):
                return data, None

            if isinstance(data, bytes):
                decoded_bytes = data
            elif isinstance(data, str):
                decoded_bytes = self._attempt_base64_decode(data)
                if decoded_bytes is None:
                    decoded_bytes = data.encode("utf-8")
            else:
                if allow_registry_lookup:
                    registry_entry = self._component_registry.get(component.name or "")
                    if registry_entry:
                        return registry_entry.get("payload"), None
                return None, "Unsupported payload format"

            payload_str = decoded_bytes.decode("utf-8")
            return json.loads(payload_str), None

        except json.JSONDecodeError as exc:
            return None, f"Failed to parse stack data: {exc}"
        except Exception as exc:
            return None, f"Unexpected error reading stack payload: {exc}"

    def _attempt_base64_decode(self, data: str) -> bytes | None:
        """Try to base64 decode a string, returning None if decoding fails."""
        try:
            return base64.b64decode(data)
        except Exception:
            return None

    def _publish_stack_action(self, method: str, payload: Any, context: str = "") -> bool:
        """Publish a stack action to the composer for execution."""
        if not self.stack_publisher:
            self.logger.error("Stack publisher is not initialized")
            return False

        try:
            payload_str = payload if isinstance(payload, str) else json.dumps(payload)

            msg_action = MutoAction()
            msg_action.context = context
            msg_action.method = method
            msg_action.payload = payload_str

            self.stack_publisher.publish(msg_action)
            self.logger.debug(
                f"Published stack action '{method}' with payload length {len(payload_str)}"
            )
            return True
        except Exception as exc:
            self.logger.error(f"Failed to publish stack action '{method}': {exc}")
            return False

    def needs_update(self, metadata: dict[str, Any], pack: ComparisonPack) -> bool:
        """
        Check if components need updates by comparing desired vs current state.

        Args:
            metadata: Request metadata.
            pack: Comparison pack containing desired and current components.

        Returns:
            True if updates are needed, False otherwise.
        """
        self.logger.info(
            f"Checking update need for {len(pack.desired)} desired vs {len(pack.current)} current"
        )

        # Create lookup for current components
        current_by_name = {comp.name: comp for comp in pack.current}

        for desired in pack.desired:
            current = current_by_name.get(desired.name)

            if not current:
                # Component doesn't exist, needs deployment
                self.logger.info(f"Component {desired.name} not found - needs deployment")
                return True

            # Check if component properties have changed
            if self._component_changed(desired, current):
                self.logger.info(f"Component {desired.name} has changes - needs update")
                return True

        # No updates needed
        self.logger.info("No components need updates")
        return False

    def needs_remove(self, metadata: dict[str, Any], pack: ComparisonPack) -> bool:
        """Check if components need removal by comparing desired vs current state."""
        self.logger.info(
            f"Checking removal need for {len(pack.desired)} desired vs {len(pack.current)} current"
        )

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
            if hasattr(self, "_register_service") and self._register_service:
                self.destroy_service(self._register_service)
            if hasattr(self, "_unregister_service") and self._unregister_service:
                self.destroy_service(self._unregister_service)
            if hasattr(self, "stack_publisher") and self.stack_publisher:
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


if __name__ == "__main__":
    exit(main())
