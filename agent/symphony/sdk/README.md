# Symphony SDK (Deprecated In Repo)

The pure Symphony SDK has moved into the standalone `symphony-sdk` package (`symphony-sdk-python/`).  The old modules in this directory now act as a compatibility shim that re-exports the external package.

```python
from symphony_sdk import SymphonyAPI, COARequest
```

Please migrate imports to `symphony_sdk` directly; `agent.symphony.sdk` will be removed in the next major release.
