# Composer Coding Standard

This document outlines the coding standards and best practices followed in the Composer project.

## Table of Contents

- [Composer Coding Standard](#composer-coding-standard)
  - [Table of Contents](#table-of-contents)
  - [General Principles](#general-principles)
  - [Python Standards](#python-standards)
    - [PEP 8 Compliance](#pep-8-compliance)
    - [Code Formatting](#code-formatting)
    - [Naming Conventions](#naming-conventions)
    - [Imports](#imports)
    - [Comments and Docstrings](#comments-and-docstrings)
    - [Type Hints](#type-hints)
    - [Error Handling and Exceptions](#error-handling-and-exceptions)
    - [Logging](#logging)
  - [ROS 2 Best Practices](#ros-2-best-practices)
    - [Node Initialization](#node-initialization)
    - [ROS 2 Interfaces](#ros-2-interfaces)
    - [Asynchronous Programming](#asynchronous-programming)
      - [Use `asyncio` When Necessary, but Ensure Compatibility with ROS 2's Threading Model](#use-asyncio-when-necessary-but-ensure-compatibility-with-ros-2s-threading-model)
      - [Do Not Use `async` Functions as ROS 2 Callbacks](#do-not-use-async-functions-as-ros-2-callbacks)
      - [Use `asyncio.run_coroutine_threadsafe` to Schedule Coroutines in the Event Loop from Synchronous Code](#use-asynciorun_coroutine_threadsafe-to-schedule-coroutines-in-the-event-loop-from-synchronous-code)
  - [Plugin Development Guidelines](#plugin-development-guidelines)
  - [Version Control Practices](#version-control-practices)
  - [Documentation Standards](#documentation-standards)
  - [Testing Guidelines](#testing-guidelines)

---

## General Principles

- **Readability Over Cleverness**: Write code that is clear and understandable rather than overly clever or terse.
- **Consistency**: Ensure consistent coding style throughout the project.
- **Modularity**: Write modular code with functions and classes that have a single responsibility.
- **Avoid Redundancy**: Do not duplicate code; instead, abstract common functionality.

## Python Standards

### PEP 8 Compliance

All Python code should adhere to [PEP 8](https://www.python.org/dev/peps/pep-0008/), the Python style guide.

### Code Formatting

- **Line Length**: Limit all lines to a maximum of 79 characters.
- **Indentation**: Use 4 spaces per indentation level.
- **Blank Lines**: Use blank lines to separate functions and classes, and larger blocks of code inside functions.
- **Whitespace**: Avoid extraneous whitespace in the following situations:
  - Immediately inside parentheses, brackets, or braces.
  - Before a comma, semicolon, or colon.
  - Before the open parenthesis that starts the argument list of a function call.

### Naming Conventions

- **Variables and Functions**: Use `lowercase_with_underscores`.
- **Classes**: Use `CapWords` (PascalCase). No parenthesis if no inheritance.
- **Constants**: Use `UPPERCASE_WITH_UNDERSCORES`.
- **Private Members**: Prefix with a single underscore `_` to indicate non-public parts of the API.
- **Modules and Packages**: Use short, all-lowercase names. Underscores can be used if necessary.

### Imports

- **Order of Imports**: Organize imports into three sections, each separated by a blank line:
  1. Standard library imports.
  2. Related third-party imports.
  3. Local application or library-specific imports.
- **Import Style**:
  - Avoid using `from module import *`.
  - Use absolute imports whenever possible.
  - Group multiple imports from the same module.

**Example:**

```python
import os
import sys

import rclpy
from rclpy.node import Node

from composer.workflow.pipeline import Pipeline
```

### Comments and Docstrings

- **Comments**:
  - Keep comments up to date with code changes.
  - Use complete sentences, starting with a capital letter and ending with a period.
  - Inline comments should be used sparingly.

- **Docstrings**:
  - All modules, classes, methods, and functions should have docstrings.
  - Describe what the code does, its parameters, return values, exceptions raised, and any side effects.

**Example:**

```python
def connect(self, timeout: int = 10) -> bool:
    """
    Connect to the server.

    Args:
        timeout (int): The timeout duration in seconds.

    Returns:
        bool: True if the connection was successful, False otherwise.

    Raises:
        ConnectionError: If the connection could not be established.
    """
    pass
```

### Type Hints

- Use type hints to specify the expected data types of variables, function arguments, and return values.

**Example:**

```python
from typing import Optional, List

def get_user(name: str) -> Optional[User]:
    pass
```

### Error Handling and Exceptions

- Use specific exception handling. Catch only exceptions that you can handle. The pipeline already catches the generic Exception class to see if something goes wrong and to abort the workflow accordingly.
- Avoid bare `except` clauses. Always specify the exception type.
- Use `raise` to re-raise exceptions after logging or performing necessary cleanup.
- Provide meaningful error messages.

**Example:**

```python
try:
    value = int(user_input)
except ValueError as e:
    logger.error(f"Invalid input '{user_input}': {e}")
    raise
```

### Logging

- Don't use Python's print.
- Use the `rclpy.logging` for non-node classes and `self.get_logger('node_name')` for `Node` classes module for logging messages. 
- Use appropriate logging levels (`debug`, `info`, `warning`, `error`).
- Include contextual information in log messages to aid debugging.

**Example:**

```python
self.logger = rclpy.logging.get_logger(__name__)
self.logger.info("Starting the process.")
```

## ROS 2 Best Practices

### Node Initialization

- Each node should inherit from `rclpy.node.Node`.
- Use meaningful node names.
- Initialize ROS 2 interfaces (publishers, subscribers, services, clients) in the `__init__` method.

**Example:**

```python
from rclpy.node import Node


class MutoNode(Node):
    def __init__(self):
        super().__init__('muto_node')
        self.publisher = self.create_publisher(String, 'muto_topic', 10)
```

### ROS 2 Interfaces

- **Publishers/Subscribers**: Use appropriate QoS profiles.
- **Services**: Ensure that service callbacks are regular functions (not `async` functions).
- **Clients**: Handle the possibility that services may not be available immediately.
- **Interfaces**: Any interface you need to define should be defined under [messages](https://github.com/eclipse-muto/messages) and the necessary build changes should be made under [`messages/CMakeLists.txt`](https://github.com/eclipse-muto/messages/blob/main/CMakeLists.txt).

### Asynchronous Programming

#### Use `asyncio` When Necessary, but Ensure Compatibility with ROS 2's Threading Model

**Explanation:**

- **`asyncio` Integration**: Python's `asyncio` library provides a framework for writing asynchronous code using the `async`/`await` syntax. It's useful for managing concurrent I/O-bound tasks without resorting to multi-threading or multi-processing.
- **ROS 2 Execution Model**: ROS 2 uses executors (`rclpy.executors.SingleThreadedExecutor` or `MultiThreadedExecutor`) to manage the execution of nodes, subscriptions, timers, and services. The executor handles callbacks in its own threading model, which may conflict with `asyncio` if not managed properly.

**Best Practices:**

- Use `asyncio` for tasks that benefit from asynchronous execution, such as network requests, file I/O, or any operation that can yield control while waiting.
- Be cautious when mixing `asyncio` and ROS 2 to avoid conflicts between their event loops and threading models.
- Ensure that `asyncio` event loops are properly managed and do not interfere with the ROS 2 executor.

#### Do Not Use `async` Functions as ROS 2 Callbacks

**Explanation:**

- **ROS 2 Callbacks**: In ROS 2, callbacks (such as service handlers, subscription callbacks, and timer callbacks) are expected to be regular synchronous functions. The ROS 2 executor invokes these callbacks as part of its event loop.
- **Limitations**: If you define an `async` function as a callback, the executor will not `await` it, leading to a `RuntimeWarning` or unexpected behavior because the coroutine is never awaited.
- **Threading Issues**: Running `async` callbacks directly can lead to concurrency issues, as the `asyncio` event loop might not be running in the same thread as the ROS 2 executor.

**Best Practices:**

- Always define ROS 2 callbacks (e.g., service handlers, subscription callbacks) as regular synchronous functions (using `def`, not `async def`).
- If you need to perform asynchronous tasks within a callback, use appropriate mechanisms to schedule the asynchronous code without blocking the callback.

**Example of What to Avoid:**

```python
# Incorrect: Defining an async function as a ROS 2 callback
async def handle_request(request, response):
    # Asynchronous code here
    await some_async_operation()
    return response

# Registering the async callback (will not work as intended)
service = node.create_service(MutoService, 'muto_service', handle_request)
```

#### Use `asyncio.run_coroutine_threadsafe` to Schedule Coroutines in the Event Loop from Synchronous Code

**Explanation:**

- **Scheduling Coroutines**: When you need to run asynchronous code from within a synchronous ROS 2 callback, you can schedule the coroutine to run in the event loop using `asyncio.run_coroutine_threadsafe`.
- **Thread Safety**: This function allows you to submit a coroutine to an event loop from another thread (which is often the case with ROS 2 callbacks), ensuring thread safety.
- **Future Object**: `asyncio.run_coroutine_threadsafe` returns a `concurrent.futures.Future` object, which you can use to check the result or handle exceptions.

**Best Practices:**

- Maintain a reference to the `asyncio` event loop in your node (e.g., `self.loop = asyncio.get_event_loop()`).
- Use `asyncio.run_coroutine_threadsafe(coroutine, loop)` to schedule the coroutine from within synchronous callbacks.
- Handle exceptions and results from the returned `Future` object if necessary.

**Example Implementation:**

```python
import asyncio
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('muto_node')
        self.loop = asyncio.get_event_loop()
        self.service = self.create_service(MyService, 'muto_service', self.handle_request)
        self.timer = self.create_timer(0.1, self.run_event_loop)

    def run_event_loop(self):
        # Periodically run the event loop to process scheduled coroutines
        self.loop.stop()
        self.loop.run_forever()

    def handle_request(self, request, response):
        # Schedule the coroutine in the event loop
        future = asyncio.run_coroutine_threadsafe(
            self.async_operation(request),
            self.loop
        )
        try:
            result = future.result(timeout=10)
            response.success = True
            response.message = result
        except Exception as e:
            self.get_logger().error(f"Async operation failed: {e}")
            response.success = False
            response.message = str(e)
        return response

    async def async_operation(self, request):
        # Asynchronous code here
        await asyncio.sleep(1)  # Simulate async I/O
        return "Operation completed"

def main():
    rclpy.init()
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

**Explanation of the Example:**

- **Event Loop Management**: The node initializes the `asyncio` event loop and runs it periodically using a timer to ensure that scheduled coroutines are executed.
- **Synchronous Callback**: The `handle_request` function is a synchronous function as required by ROS 2.
- **Scheduling the Coroutine**: Inside the callback, `asyncio.run_coroutine_threadsafe` is used to schedule the `async_operation` coroutine.
- **Handling Results**: The result of the coroutine is retrieved using `future.result(timeout=10)`, which waits for the coroutine to complete or raises a timeout.
- **Exception Handling**: Exceptions are caught and logged, and appropriate responses are set.

---

**Key Takeaways:**

- **Avoid Blocking the Executor**: Do not perform long-running or blocking operations directly within ROS 2 callbacks, as this can block the executor and prevent it from handling other callbacks.
- **Thread Safety**: Be mindful of thread safety when interacting with the `asyncio` event loop from ROS 2 callbacks, which may run in different threads.
- **Event Loop Management**: Ensure that the `asyncio` event loop is running and accessible where needed. You may need to create and manage the event loop within your node.
- **Testing and Debugging**: As asynchronous code can introduce complexity, thoroughly test and debug your asynchronous integrations to ensure they work as expected.

---

**Common Pitfalls and How to Avoid Them:**

1. **Using `async` Callbacks Directly**:

   - **Issue**: Defining `async def` callbacks and expecting the ROS 2 executor to handle them asynchronously.
   - **Solution**: Always define callbacks as synchronous functions and schedule asynchronous tasks using the methods described.

2. **Event Loop Not Running**:

   - **Issue**: Scheduling coroutines in an event loop that is not running, leading to coroutines never being executed.
   - **Solution**: Ensure that the `asyncio` event loop is running, either by integrating it with the ROS 2 executor or by running it periodically using a timer.

3. **Threading Conflicts**:

   - **Issue**: Accessing shared resources from both the ROS 2 executor and asynchronous coroutines without proper synchronization.
   - **Solution**: Use thread-safe mechanisms (e.g., locks) or ensure that shared resources are only accessed from one thread.

4. **Exception Handling**:

   - **Issue**: Unhandled exceptions in coroutines can be difficult to debug and may cause unexpected behavior.
   - **Solution**: Always handle exceptions in coroutines and when retrieving results from futures.

---

Remember to:

- Keep ROS 2 callbacks synchronous.
- Use `asyncio.run_coroutine_threadsafe` to schedule asynchronous tasks safely.
- Manage the `asyncio` event loop appropriately within your nodes.
- Handle exceptions and be cautious of threading issues.

## Plugin Development Guidelines

- Plugins should be modular and encapsulated.
- Each plugin should have a clear purpose and responsibility.
- Plugins must have corresponding service definitions defined under [messages](https://github.com/eclipse-muto/messages).
- Follow the same coding standards and ROS 2 best practices as the main codebase.

## Version Control Practices

- **Branching Model**: Use feature branches for new development and merge back into the main branch after review.
- **Commit Messages**: Write clear and concise commit messages that describe what changes were made and why. Try to follow [conventional commit style](https://www.conventionalcommits.org/en/v1.0.0/)

## Documentation Standards

- Document any architectural decisions or significant design patterns used.
- Provide usage examples where appropriate.

## Testing Guidelines

- Write unit tests for new code using `unittest` library.
- Ensure tests cover critical functionality and edge cases.
- Use continuous integration to run tests automatically on new commits.
- Test coverage must always stay at >=80%

---
