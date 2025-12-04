"""Deprecated shim that forwards to the standalone `symphony_sdk` package."""

from __future__ import annotations

import warnings

from symphony_sdk import *  # type: ignore F401,F403
from symphony_sdk import __all__ as _sdk_all

warnings.warn(
    "agent.symphony.sdk is deprecated and will be removed in a future release. "
    "Please import from the standalone symphony_sdk package instead.",
    DeprecationWarning,
    stacklevel=2,
)

__all__ = list(_sdk_all)
