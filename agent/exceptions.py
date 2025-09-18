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