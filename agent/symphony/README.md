# SDK Package

The `agent.symphony.sdk` package contains an SDK for Symphony data structures and utilities:

- **symphony_api.py**: REST API client and authentication
- **symphony_sdk.py**: COA protocol, data structures, serialization
- **symphony_summary.py**: Deployment summary and result models  
- **symphony_types.py**: State enums and constants

## Package Structure

```
src/agent/agent/symphony/
├── __init__.py              # Package exports and convenience imports
├── symphony_broker.py       # MQTT broker integration
├── symphony_provider.py     # Main Symphony provider implementation
└── sdk/                     # Core SDK components
    ├── __init__.py          # SDK exports and convenience imports
    ├── symphony_api.py      # REST API client for Symphony
    ├── symphony_sdk.py      # COA data structures and SDK
    ├── symphony_summary.py  # Summary models and result handling
    └── symphony_types.py    # State enums and constants
```

## Import 
```python
# Convenient imports from main symphony package
from agent.symphony import COARequest, COAResponse, State

# SDK-specific imports
from agent.symphony.sdk import SummaryResult, SymphonyAPIClient

# Provider/broker imports
from agent.symphony.symphony_provider import MutoSymphonyProvider

# Direct module access
from agent.symphony.sdk.symphony_types import State as SymphonyState
```


### SDK Usage Examples

```python
# Import everything from SDK
from agent.symphony.sdk import *

# Create COA request/response
request = COARequest(method='GET', route='/targets')
response = COAResponse.success({'targets': []})

# Use summary models
summary = SummarySpec(target_count=1, success_count=1)

# API client usage  
client = SymphonyAPIClient(base_url='http://localhost:8082/v1alpha2/')
```