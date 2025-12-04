# Symphony Integration Package

The core Symphony SDK has been extracted into the standalone [`symphony-sdk`](../../symphony-sdk-python) project.  The Muto agent now depends on that package and only keeps the provider/broker implementation locally.

## Current Layout

```
src/agent/agent/symphony/
├── __init__.py              # Re-exports from symphony_sdk + provider shim
├── provider_base.py         # Local abstract SymphonyProvider interface
├── symphony_broker.py       # MQTT broker integration
├── symphony_provider.py     # Muto-specific provider
└── sdk/__init__.py          # Deprecation shim (imports symphony_sdk)
```

## Import Examples

```python
from symphony_sdk import SymphonyAPI, SummaryResult, State
from agent.symphony.provider_base import SymphonyProvider
from agent.symphony.symphony_provider import MutoSymphonyProvider
```

Use `pip install symphony-sdk` (or the bundled `symphony-sdk-python` directory) to access the COA models, REST client, and summary/types modules.
