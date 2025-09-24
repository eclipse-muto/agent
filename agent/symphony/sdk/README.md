# Symphony SDK

Core SDK components for Symphony integration.

## Components

### symphony_api.py
- **SymphonyAPIClient**: REST API client for Symphony operations
- **SymphonyAPIError**: Custom exception for API errors
- Authentication and target management

### symphony_sdk.py  
- **COA Protocol**: COARequest, COAResponse classes
- **Data Models**: ComponentSpec, TargetSpec, DeploymentSpec, etc.
- **Serialization**: JSON serialization/deserialization utilities
- **SymphonyProvider**: Abstract base class for providers

### symphony_summary.py
- **SummaryResult**: Complete deployment summary container
- **SummarySpec**: Deployment statistics and target results  
- **ComponentResultSpec**: Individual component operation results
- **TargetResultSpec**: Target-level operation results
- **SummaryState**: Enumeration for summary states

### symphony_types.py
- **State**: Comprehensive state enum matching Symphony Go implementation
- **COAConstants**: Protocol constants and configuration keys
- **Terminable**: Interface for graceful shutdown
- **Utility functions**: HTTP status code mapping

## Quick Start

```python
from agent.symphony.sdk import *

# Create a COA request
request = COARequest(
    method='POST',
    route='/targets/deploy', 
    content_type='application/json'
)
request.set_body({'components': [{'name': 'app', 'type': 'container'}]})

# Create a response
response = COAResponse.success({'deployed': True})

# Use summary models
summary = SummarySpec(target_count=1, success_count=1)
result = SummaryResult(summary=summary, state=SummaryState.DONE)

# API operations
client = SymphonyAPIClient(base_url='http://symphony:8082/v1alpha2/')
# Use client for Symphony operations...
```

## Import Patterns

```python
# Import from SDK root (recommended)
from agent.symphony.sdk import COARequest, COAResponse, State

# Import specific modules  
from agent.symphony.sdk.symphony_api import SymphonyAPIClient
from agent.symphony.sdk.symphony_summary import SummaryResult

# Import everything (use carefully)
from agent.symphony.sdk import *
```