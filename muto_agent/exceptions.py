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
Core exceptions for the Muto Agent system.
"""


class MutoAgentError(Exception):
    """Base exception class for Muto Agent errors."""

    pass


class ConnectionError(MutoAgentError):
    """Raised when connection to external services fails."""

    pass


class ConfigurationError(MutoAgentError):
    """Raised when configuration is invalid or missing."""

    pass


class MessageParsingError(MutoAgentError):
    """Raised when message parsing fails."""

    pass


class ServiceNotReadyError(MutoAgentError):
    """Raised when a required service is not ready."""

    pass


class CommandNotFoundError(MutoAgentError):
    """Raised when a requested command is not found."""

    pass


class TopicParsingError(MessageParsingError):
    """Raised when topic parsing fails."""

    pass


class InvalidTopicError(MutoAgentError):
    """Raised when topic format is invalid."""

    pass
