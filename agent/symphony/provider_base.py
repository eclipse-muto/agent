"""Abstract Symphony provider interfaces that remain inside the agent."""

from typing import Any, Dict, List

from symphony_sdk import ComponentSpec, ComparisonPack


class SymphonyProvider:
    """Abstract base class implemented by concrete Symphony providers."""

    def init_provider(self) -> None:
        """Initialize the provider with configuration or services it needs."""
        raise NotImplementedError("init_provider must be implemented by subclasses")

    def apply(self, metadata: Dict[str, Any], components: List[ComponentSpec]) -> str:
        """Apply/deploy the desired component state to the target."""
        raise NotImplementedError("apply must be implemented by subclasses")

    def remove(self, metadata: Dict[str, Any], components: List[ComponentSpec]) -> str:
        """Remove components from the target."""
        raise NotImplementedError("remove must be implemented by subclasses")

    def get(self, metadata: Dict[str, Any], components: List[ComponentSpec]) -> str:
        """Return the current state for the requested components."""
        raise NotImplementedError("get must be implemented by subclasses")

    def needs_update(self, metadata: Dict[str, Any], pack: ComparisonPack) -> bool:
        """Return True if an update needs to run for the provided comparison data."""
        raise NotImplementedError("needs_update must be implemented by subclasses")

    def needs_remove(self, metadata: Dict[str, Any], pack: ComparisonPack) -> bool:
        """Return True if components must be removed for the provided comparison data."""
        raise NotImplementedError("needs_remove must be implemented by subclasses")


__all__ = ['SymphonyProvider']
