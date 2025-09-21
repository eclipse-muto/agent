"""
Symphony SDK Package.

This package contains core Symphony SDK components including:
- Data structures and COA protocol
- REST API client
- Summary models  
- Type definitions
"""

# Import core SDK components for convenience
from .symphony_api import (
    SymphonyAPIClient,
    SymphonyAPIError,
)
from .symphony_sdk import (
    COARequest,
    COAResponse,
    ComponentSpec,
    DeploymentSpec,
    SolutionSpec,
    SymphonyProvider,
    TargetSpec,
    deserialize_coa_request,
    deserialize_coa_response,
    from_dict,
    serialize_coa_request,
    serialize_coa_response,
    to_dict,
)
from .symphony_summary import (
    ComponentResultSpec,
    SummaryResult,
    SummarySpec,
    SummaryState,
    TargetResultSpec,
    create_failed_component_result,
    create_success_component_result,
    create_target_result,
)
from .symphony_types import (
    COAConstants,
    State,
    Terminable,
    get_http_status,
)

# Export all SDK components
__all__ = [
    # API client
    'SymphonyAPIClient',
    'SymphonyAPIError',
    
    # Core SDK
    'COARequest',
    'COAResponse',
    'ComponentSpec',
    'DeploymentSpec',
    'SolutionSpec',
    'SymphonyProvider',
    'TargetSpec',
    'deserialize_coa_request',
    'deserialize_coa_response',
    'from_dict',
    'serialize_coa_request',
    'serialize_coa_response',
    'to_dict',
    
    # Summary models
    'ComponentResultSpec',
    'SummaryResult',
    'SummarySpec',
    'SummaryState',
    'TargetResultSpec',
    'create_failed_component_result',
    'create_success_component_result',
    'create_target_result',
    
    # Types
    'COAConstants',
    'State',
    'Terminable',
    'get_http_status',
]