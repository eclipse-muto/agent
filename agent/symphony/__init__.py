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

# Import commonly used components for convenience
from .sdk.symphony_sdk import (
    COARequest,
    COAResponse,
    ComponentSpec,
    DeploymentSpec,
    SolutionSpec,
    SymphonyProvider,
    TargetSpec,
    deserialize_coa_request,
    deserialize_coa_response,
    serialize_coa_request,
    serialize_coa_response,
)
from .sdk.symphony_summary import (
    ComponentResultSpec,
    SummaryResult,
    SummarySpec,
    SummaryState,
    TargetResultSpec,
)
from .sdk.symphony_types import (
    COAConstants,
    State,
    Terminable,
    get_http_status,
)

# Export main classes for external use
__all__ = [
    # SDK components
    'COARequest',
    'COAResponse',
    'ComponentSpec',
    'DeploymentSpec',
    'SolutionSpec',
    'SymphonyProvider',
    'TargetSpec',
    'deserialize_coa_request',
    'deserialize_coa_response',
    'serialize_coa_request',
    'serialize_coa_response',
    
    # Types
    'COAConstants',
    'State',
    'Terminable',
    'get_http_status',
    
    # Summary models
    'ComponentResultSpec',
    'SummaryResult',
    'SummarySpec',
    'SummaryState',
    'TargetResultSpec',
]