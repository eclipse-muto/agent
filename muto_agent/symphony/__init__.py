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
    COARequest,
    COAResponse,
    ComponentSpec,
    DeploymentSpec,
    SolutionSpec,
    TargetSpec,
    ComponentResultSpec,
    SummaryResult,
    SummarySpec,
    SummaryState,
    TargetResultSpec,
    COAConstants,
    State,
    Terminable,
    get_http_status,
    deserialize_coa_request,
    deserialize_coa_response,
    serialize_coa_request,
    serialize_coa_response,
)
from .provider_base import SymphonyProvider

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
