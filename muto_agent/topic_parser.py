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
Topic parsing utilities for the Muto Agent system.
"""

import re
from typing import Any

from .exceptions import InvalidTopicError, TopicParsingError
from .interfaces import TopicParser


class MutoTopicParser(TopicParser):
    """
    Parser for Muto protocol topics.

    This class handles parsing of various topic formats used in the Muto
    protocol, extracting relevant information like type and method.
    """

    # Topic patterns for different message types
    STACK_PATTERN = r".*/stack/commands/(.*)"
    AGENT_PATTERN = r".*/agent/commands/(.*)"
    THINGS_PATTERN = r".*/things/([^/]*)/([^/]*)/(.*)"

    def __init__(self, logger: Any | None = None):
        """Initialize the topic parser.

        Args:
            logger: Optional logger instance. If None, logging will be skipped.
        """
        self._logger = logger

    def parse_topic(self, topic: str) -> tuple[str | None, str | None]:
        """
        Parse topic from gateway message.

        This method parses topic and detects type and method.

        Args:
            topic: Topic from gateway message.

        Returns:
            A tuple (type, method), where type is either stack, agent, ping, etc.
            and method is command to run. Will return (None, None) tuple if topic
            doesn't match any known patterns.

        Raises:
            TopicParsingError: If topic parsing fails due to invalid format.
        """
        if not topic or not isinstance(topic, str):
            raise InvalidTopicError("Topic must be a non-empty string")

        try:
            # Handle telemetry topics
            if "telemetry" in topic:
                return None, None

            # Handle ping topics
            elif "ping" in topic:
                return "ping", None

            # Handle stack topics
            elif "stack" in topic:
                match = re.findall(self.STACK_PATTERN, topic)
                if match:
                    return "stack", match[0]
                else:
                    raise TopicParsingError(f"Invalid stack topic format: {topic}")

            # Handle agent topics
            elif "agent" in topic:
                match = re.findall(self.AGENT_PATTERN, topic)
                if match:
                    return "agent", match[0]
                else:
                    raise TopicParsingError(f"Invalid agent topic format: {topic}")

            # Unknown topic type
            else:
                if self._logger:
                    self._logger.debug(f"Unknown topic type: {topic}")
                return None, None

        except re.error as e:
            raise TopicParsingError(f"Regex parsing failed for topic '{topic}': {e}") from e
        except Exception as e:
            raise TopicParsingError(f"Failed to parse topic '{topic}': {e}") from e

    def parse_things_topic(self, topic: str) -> tuple[str | None, str | None, list | None]:
        """
        Parse Ditto things protocol topic.

        Args:
            topic: The things protocol topic string.

        Returns:
            A tuple of (channel, criterion, action_parts) or (None, None, None) if parsing fails.

        Raises:
            TopicParsingError: If topic parsing fails.
        """
        try:
            match = re.findall(self.THINGS_PATTERN, topic)
            if match and len(match[0]) >= 3:
                channel = match[0][0]
                criterion = match[0][1]
                action_parts = match[0][2].split("/")
                return channel, criterion, action_parts
            else:
                raise TopicParsingError(f"Invalid things topic format: {topic}")

        except re.error as e:
            raise TopicParsingError(f"Regex parsing failed for things topic '{topic}': {e}") from e
        except Exception as e:
            raise TopicParsingError(f"Failed to parse things topic '{topic}': {e}") from e

    def is_valid_topic(self, topic: str) -> bool:
        """
        Check if a topic is valid.

        Args:
            topic: The topic string to validate.

        Returns:
            True if topic is valid, False otherwise.
        """
        try:
            result = self.parse_topic(topic)
            return result != (None, None)
        except (TopicParsingError, InvalidTopicError):
            return False

    def extract_command_from_topic(self, topic: str) -> str | None:
        """
        Extract command name from a topic.

        Args:
            topic: The topic string.

        Returns:
            The extracted command name or None if not found.
        """
        try:
            _, method = self.parse_topic(topic)
            return method
        except (TopicParsingError, InvalidTopicError):
            return None
