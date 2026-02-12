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

from __future__ import annotations

"""
Base interfaces and abstract classes for the Muto Agent system.
"""

from abc import ABC, abstractmethod
from typing import Any, Dict, Optional
from rclpy.node import Node


class MessageHandler(ABC):
    """Abstract base class for message handlers."""
    
    @abstractmethod
    def handle_message(self, data: Any) -> None:
        """
        Handle an incoming message.
        
        Args:
            data: The message data to handle.
        """
        pass


class TopicParser(ABC):
    """Abstract base class for topic parsers."""
    
    @abstractmethod
    def parse_topic(self, topic: str) -> tuple[Optional[str], Optional[str]]:
        """
        Parse a topic string to extract type and method.
        
        Args:
            topic: The topic string to parse.
            
        Returns:
            A tuple of (type, method) or (None, None) if parsing fails.
        """
        pass


class MessagePublisher(ABC):
    """Abstract base class for message publishers."""
    
    @abstractmethod
    def publish(self, message: Any) -> None:
        """
        Publish a message.
        
        Args:
            message: The message to publish.
        """
        pass


class ServiceClient(ABC):
    """Abstract base class for service clients."""
    
    @abstractmethod
    def call_service(self, request: Any) -> Any:
        """
        Call a service with the given request.
        
        Args:
            request: The service request.
            
        Returns:
            The service response.
        """
        pass


class ConnectionManager(ABC):
    """Abstract base class for connection managers."""
    
    @abstractmethod
    def connect(self) -> bool:
        """
        Establish connection.
        
        Returns:
            True if connection successful, False otherwise.
        """
        pass
    
    @abstractmethod
    def disconnect(self) -> None:
        """Disconnect from the service."""
        pass
    
    @abstractmethod
    def is_connected(self) -> bool:
        """
        Check if currently connected.
        
        Returns:
            True if connected, False otherwise.
        """
        pass


class CommandExecutor(ABC):
    """Abstract base class for command executors."""
    
    @abstractmethod
    def execute_command(self, method: str, payload: str, meta: Any) -> Any:
        """
        Execute a command.
        
        Args:
            method: The command method to execute.
            payload: The command payload.
            meta: Metadata for the command.
            
        Returns:
            The command result.
        """
        pass


class ResourceManager(ABC):
    """Abstract base class for resource managers."""
    
    @abstractmethod
    def initialize(self) -> None:
        """Initialize resources."""
        pass
    
    @abstractmethod
    def cleanup(self) -> None:
        """Clean up resources."""
        pass


class BaseNode(Node, ResourceManager):
    """
    Enhanced base class for ROS nodes with proper resource management.
    
    This class provides common functionality for all nodes in the system,
    including proper initialization, cleanup, and error handling.
    """
    
    def __init__(self, node_name: str, **kwargs):
        """
        Initialize the base node.
        
        Args:
            node_name: Name of the ROS node.
            **kwargs: Additional keyword arguments for Node initialization.
        """
        super().__init__(node_name, **kwargs)
        self._initialized = False
        self._resources_cleaned = False
    
    def initialize(self) -> None:
        """Initialize the node resources."""
        if self._initialized:
            return
            
        try:
            self._do_initialize()
            self._initialized = True
            self.get_logger().info(f"Node {self.get_name()} initialized successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize node {self.get_name()}: {e}")
            raise
    
    def cleanup(self) -> None:
        """Clean up node resources."""
        if self._resources_cleaned:
            return
            
        try:
            self._do_cleanup()
            self._resources_cleaned = True
            # Only log if the node handle is still valid
            try:
                self.get_logger().info(f"Node {self.get_name()} cleaned up successfully")
            except:
                # Node handle is already destroyed, skip logging
                pass
        except Exception as e:
            try:
                self.get_logger().error(f"Failed to cleanup node {self.get_name()}: {e}")
            except:
                # Node handle is already destroyed, skip logging
                pass
    
    def __del__(self):
        """Destructor to ensure cleanup."""
        # Skip cleanup in destructor to avoid ROS handle issues
        # Cleanup should be called explicitly in tests
        pass
    
    @abstractmethod
    def _do_initialize(self) -> None:
        """Subclass-specific initialization logic."""
        pass
    
    @abstractmethod
    def _do_cleanup(self) -> None:
        """Subclass-specific cleanup logic."""
        pass