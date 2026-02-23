"""
Symphony Integration Package for Muto Agent.

This package contains all Symphony-related components including:
- API client
- MQTT broker integration
- Provider implementation
- SDK data structures
- Summary models
- Type definitions
"""

from symphony_sdk import (
    COAConstants,
    COARequest,
    COAResponse,
    ComponentResultSpec,
    ComponentSpec,
    DeploymentSpec,
    SolutionSpec,
    State,
    SummaryResult,
    SummarySpec,
    SummaryState,
    TargetResultSpec,
    TargetSpec,
    Terminable,
    deserialize_coa_request,
    deserialize_coa_response,
    get_http_status,
    serialize_coa_request,
    serialize_coa_response,
)

from .provider_base import SymphonyProvider

# Export main classes for external use
__all__ = [
    # SDK components
    "COARequest",
    "COAResponse",
    "ComponentSpec",
    "DeploymentSpec",
    "SolutionSpec",
    "SymphonyProvider",
    "TargetSpec",
    "deserialize_coa_request",
    "deserialize_coa_response",
    "serialize_coa_request",
    "serialize_coa_response",
    # Types
    "COAConstants",
    "State",
    "Terminable",
    "get_http_status",
    # Summary models
    "ComponentResultSpec",
    "SummaryResult",
    "SummarySpec",
    "SummaryState",
    "TargetResultSpec",
]
