# Symphony SDK for Python

`symphony-sdk` is a lightweight, dependency-minimal Python client for interacting with Eclipse Symphony.  It bundles the REST API client together with the COA data models, type definitions, and summary helpers used by Symphony control planes and providers.

## Features

- Modern REST client with token management and helpful exceptions (`SymphonyAPI`)
- COA data models for targets, deployments, components, and COA request/response helpers
- Summary utilities mirroring the upstream Symphony reference implementation
- Dependency-minimal: only requires `requests`

## Installation

```bash
pip install symphony-sdk
```

## Quick Start

```python
from symphony_sdk import SymphonyAPI, InstanceSpec

client = SymphonyAPI(
    base_url="https://symphony.example.com",
    username="user",
    password="pass",
)

instance_spec = InstanceSpec(name="demo", solution="solution-name")
response = client.create_instance(instance_spec)
```

See the `examples/` directory and the unit tests for additional usage patterns.

## Development

```bash
pip install -e .
pytest
```

## License

Dual-licensed under EPL-2.0 **or** MIT.  See `LICENSE` for details.
