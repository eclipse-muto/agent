# Muto Agent

| ROS 2 Distro | Ubuntu | Python | Status |
|---|---|---|---|
| Humble | 22.04 | 3.10 | [![Humble](https://github.com/ibrahimsel/agent/actions/workflows/ci-humble.yaml/badge.svg)](https://github.com/ibrahimsel/agent/actions/workflows/ci-humble.yaml) |
| Jazzy | 24.04 | 3.12 | [![Jazzy](https://github.com/ibrahimsel/agent/actions/workflows/ci-jazzy.yaml/badge.svg)](https://github.com/ibrahimsel/agent/actions/workflows/ci-jazzy.yaml) |

**Muto Agent** is the agnostic communication bridge within Eclipse Muto's declarative orchestrator for ROS software stacks on automotive edge devices. As the robot's secure gateway to any cloud backend, it features a pluggable architecture that makes Muto fundamentally agnostic to external systems, enabling interoperability with various cloud orchestrators and communication standards.

## Overview

The Agent is Eclipse Muto's **agnostic communication bridge** - the robot's secure gateway to any cloud backend. Its defining characteristic is a **pluggable architecture** that decouples Muto's core logic from particular cloud orchestrators or communication standards, providing **interoperability** and **flexibility**.

The Agent's primary function is to securely and reliably bridge the vehicle and the cloud, delivering the desired **"state"** from any backend system to the Composer for on-vehicle reconciliation. By default, it comes with Eclipse Ditto and MQTT plugins, but can easily be extended to work with other systems like Eclipse Symphony, Zenoh, or uProtocol through its plugin system.


## Eclipse Muto: Core Concepts & Function

Eclipse Muto is a **declarative orchestrator** for managing ROS software stacks on automotive edge devices. Its core function is to ensure the software running on a vehicle continuously matches a predefined **"desired state"**. This declarative model allows operators to define _what_ the final software configuration should be, while Muto's components autonomously handle _how_ to achieve and maintain it.

---

## Architectural Concepts

Muto's architecture is built on two primary modules that work in tandem: the Agent and the Composer.

### Agent: The Agnostic Communication Bridge

The **Agent** is the robot's secure gateway to any cloud backend. Its defining feature is a **pluggable architecture**. This design makes Muto fundamentally agnostic to the external systems it communicates with. By default, the Agent comes with a (Eclipse Ditto)[] and mqtt plugin, but can easily be made to work with others such as (Eclipse Symphony)[], Zenoh or uProtocol.

* A configured **"Plugin"** handles the specific details of the transport protocol (like MQTT or gRPC) and the payload schema (like a specific digital twin format).
* This decouples Muto's core logic from praticular cloud orchestrator or communication standard, providing **interoperability** and **flexibility**.

The Agent's primary function is to securely and reliably bridge the vehicle and the cloud, delivering the desired **"state"** to the Composer.

### Composer: The On-Vehicle Reconciliation Engine

The **Composer** is the intelligent engine that enforces the desired state on the vehicle. It operates a continuous **reconciliation loop** which is the heart of the orchestration process:

1.  **Inspects** the current state of the live ROS system.
2.  **Compares** this live state to the desired state model received from the Agent.
3.  **Acts** to close any gap by executing the precise lifecycle  using a **"pipeline"** of commands (start, stop, reconfigure) on the ROS nodes.

This loop ensures the vehicle's software configuration is self-healing and always converging towards the intended state.

---

In essence, the Agent provides the _what_ (the desired state from any backend), and the Composer handles the _how_ (the on-vehicle actions to achieve that state). This separation of concerns creates a robust and adaptable system for the dynamic environment of a software-defined vehicle.

## Muto Modules

The platform consists of four main modules:
- **Agent**: Communication bridge and command execution hub (this package)
- **Core**: Digital twin synchronization and state management
- **Composer**: Software deployment and lifecycle orchestration
- **Messages**: Shared message definitions and communication protocols

This documentation provides an overview about Agent. Even though you could use Agent alone with some little tweaks, it was intended to be used with other parts of [`Eclipse Muto`](https://eclipse-muto.github.io/docs/docs/muto/). They could be found under [`Core`](https://github.com/eclipse-muto/core), [`Composer`](https://github.com/eclipse-muto/composer) and [`Messages`](https://github.com/eclipse-muto/messages). You could refer to [`Eclipse Muto`](https://eclipse-muto.github.io/docs/docs/muto/) documentation for a detailed guide on how to set Muto up for your use-case.

## Agent Features

- **Eclipse Ditto/MQTT Gateway**: Secure bidirectional communication with cloud-based digital twin systems
- **Command Execution Framework**: Plugin-based architecture for executing remote commands
- **Message Routing**: Intelligent routing of messages between Muto components
- **Configuration Management**: Dynamic parameter handling integrated with ROS 2 parameter system
- **Parsing**: Flexible command/topic structure parsing for different **"Plugin"** deployments
- **ROS 2 Integration**: Native integration with ROS 2 ecosystem for seamless robotics development

## Agent Structure

The Agent package consists of the following main components:

- **Core Agent**
  - **Muto Agent**: Main coordinator node managing component lifecycle and message routing
  - **Ditto/MQTT Gateway**: Handles secure cloud connectivity and message translation
  - **Message Handlers**: Process and route different message types between components
- **Command System**
  - **Command Executor**: Framework for executing commands through plugins
  - **Command Registry**: Manages available commands and their configurations
  - **Command Plugins**: Extensible plugins for different command types (ROS tools, system commands, etc.)
- **Configuration Management**
  - **Config Manager**: Handles parameter loading and validation
  - **Parser**: Parses and manages topic structures for multi-vehicle scenarios

The Agent acts as a central communication hub, receiving messages from the cloud via MQTT, routing them to appropriate Muto components (Core, Composer), executing commands locally, and sending responses back to the digital twin.


## Quick Start

### Prerequisites

- **ROS 2 Humble** or later installed on your system.
- **Ditto/MQTT Broker** access for cloud connectivity.


### Steps

1. **Clone the Repository**

   ```bash
   cd $HOME
   mkdir -p muto/src
   cd muto/src
   git clone https://github.com/eclipse-muto/agent.git
   git clone https://github.com/eclipse-muto/core.git
   git clone https://github.com/eclipse-muto/composer.git
   git clone https://github.com/eclipse-muto/messages.git
   ```

2. **Install Dependencies**

   Use `rosdep` to install dependencies:

   ```bash
   cd $HOME/muto
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the Package**

   Use `colcon` to build the package:

   ```bash
   cd $HOME/muto
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

## Usage

### Configuration

- For a better workflow, you need to create 2 files: `muto.yaml`, and `muto.launch.py`

```bash
cd $HOME/muto
mkdir config/ && cd config/
# Create the below muto.yaml file under this config directory
```

`muto.yaml`:
```yaml
/**:
  ros__parameters:

    prefix: muto
    #You can override this during launch using vehicle_namespace argument
    namespace: org.eclipse.muto.sandbox    
    #You can override this during launch using vehicle_name argument
    name: mytest_vehicle_001

    stack_topic: "stack"
    twin_topic: "twin"
    agent_to_gateway_topic: "agent_to_gateway"
    gateway_to_agent_topic: "gateway_to_agent"
    agent_to_commands_topic: "agent_to_command"
    commands_to_agent_topic: "command_to_agent"
    thing_messages_topic: "thing_messages"
    ignored_packages: ["package1", "package3"]  # the packages in this list will be ignored in the build phase

    twin_url: "http://ditto:ditto@sandbox.composiv.ai"
    host: sandbox.composiv.ai
    port: 1883
    keep_alive: 60
    user: null
    password: null
    anonymous: false
    type: real_car
    attributes: '{"brand": "muto", "model": "agent"}'

    commands:
      command1:
        name: ros/topic
        service: rostopic_list
        plugin: CommandPlugin

      command2:
        name: ros/topic/info
        service: rostopic_info
        plugin: CommandPlugin

      command3:
        name: ros/topic/echo
        service: rostopic_echo
        plugin: CommandPlugin

      command4:
        name: ros/node
        service: rosnode_list
        plugin: CommandPlugin

      command5:
        name: ros/node/info
        service: rosnode_info
        plugin: CommandPlugin

      command6:
        name: ros/param
        service: rosparam_list
        plugin: CommandPlugin

      command7:
        name: ros/param/get
        service: rosparam_get
        plugin: CommandPlugin
```

```bash
cd $HOME/muto
mkdir launch/ && cd launch/
# Create the below muto.launch.py file under this launch directory
```

`muto.launch.py`:
```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions.include_launch_description import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Arguments
    muto_namespace_arg = DeclareLaunchArgument("muto_namespace", default_value="muto")
    vehicle_namespace_arg = DeclareLaunchArgument(
        "vehicle_namespace",
        default_value="org.eclipse.muto.sandbox",
        description="Vehicle ID namespace",
    )
    vehicle_name_arg = DeclareLaunchArgument(
        "vehicle_name", description="Vehicle Name"
    )

    # Files
    muto_params = "config/muto.yaml"

    # Agent
    node_agent = Node(
        namespace=LaunchConfiguration("muto_namespace"),
        name="agent",
        package="agent",
        executable="muto_agent",
        output="screen",
        parameters=[
            muto_params,
            {"namespace": LaunchConfiguration("vehicle_namespace")},
            {"name": LaunchConfiguration("vehicle_name")},
        ],
    )

    node_mqtt_gateway = Node(
        namespace=LaunchConfiguration("muto_namespace"),
        name="gateway",
        package="agent",
        executable="mqtt",
        output="screen",
        parameters=[
            muto_params,
            {"namespace": LaunchConfiguration("vehicle_namespace")},
            {"name": LaunchConfiguration("vehicle_name")},
        ],
    )

    node_commands = Node(
        namespace=LaunchConfiguration("muto_namespace"),
        name="commands_plugin",
        package="agent",
        executable="commands",
        output="screen",
        parameters=[
            muto_params,
            {"namespace": LaunchConfiguration("vehicle_namespace")},
            {"name": LaunchConfiguration("vehicle_name")},
        ],
    )

    # Core
    node_twin = Node(
        namespace=LaunchConfiguration("muto_namespace"),
        name="core_twin",
        package="core",
        executable="twin",
        output="screen",
        parameters=[
            muto_params,
            {"namespace": LaunchConfiguration("vehicle_namespace")},
            {"name": LaunchConfiguration("vehicle_name")},
        ],
    )

    # Composer
    node_composer = Node(
        namespace=LaunchConfiguration("muto_namespace"),
        name="muto_composer",
        package="composer",
        executable="muto_composer",
        output="screen",
        parameters=[
            muto_params,
            {"namespace": LaunchConfiguration("vehicle_namespace")},
            {"name": LaunchConfiguration("vehicle_name")},
        ],
    )

    node_compose_plugin = Node(
        namespace=LaunchConfiguration("muto_namespace"),
        name="compose_plugin",
        package="composer",
        executable="compose_plugin",
        output="screen",
        parameters=[
            muto_params,
            {"namespace": LaunchConfiguration("vehicle_namespace")},
            {"name": LaunchConfiguration("vehicle_name")},
        ],
    )

    node_provision_plugin = Node(
        namespace=LaunchConfiguration("muto_namespace"),
        name="provision_plugin",
        package="composer",
        executable="provision_plugin",
        output="screen",
        parameters=[
            muto_params,
            {"namespace": LaunchConfiguration("vehicle_namespace")},
            {"name": LaunchConfiguration("vehicle_name")},
        ],
    )

    node_launch_plugin = Node(
        namespace=LaunchConfiguration("muto_namespace"),
        name="launch_plugin",
        package="composer",
        executable="launch_plugin",
        output="screen",
        parameters=[
            muto_params,
            {"namespace": LaunchConfiguration("vehicle_namespace")},
            {"name": LaunchConfiguration("vehicle_name")},
        ],
    )

    # Launch Description Object
    ld = LaunchDescription()

    ld.add_action(muto_namespace_arg)
    ld.add_action(vehicle_namespace_arg)
    ld.add_action(vehicle_name_arg)

    ld.add_action(node_agent)
    ld.add_action(node_mqtt_gateway)
    ld.add_action(node_commands)
    ld.add_action(node_twin)
    ld.add_action(node_composer)
    ld.add_action(node_compose_plugin)
    ld.add_action(node_provision_plugin)
    ld.add_action(node_launch_plugin)

    return ld
```

- Agent uses the configuration file `muto.yaml` to define MQTT connection parameters, topic mappings, and available commands. Ensure that this file is properly configured for your deployment environment.

### Launching

To start `Muto` as a whole (including `agent`):

```bash
cd $HOME/muto
source /opt/ros/$ROS_DISTRO/setup.bash && source install/local_setup.bash
ros2 launch launch/muto.launch.py vehicle_namespace:=org.eclipse.muto.test vehicle_name:=test-robot-$(shuf -i 1000-9999 -n 1)
```

### Sending Commands

Agent listens for command execution requests through ROS 2 services and MQTT messages. Commands can be sent from the digital twin via MQTT or directly through ROS 2 service calls.

```bash
# Example: List available ROS topics
ros2 service call /muto/agent/execute_command muto_msgs/srv/CommandPlugin "{method: 'ros/topic', payload: '', meta: {}}"
```

## Plugins

Agent's command execution functionality can be extended through plugins. The default command plugins provide:

- **ROS Topic Commands**: List, info, and echo operations on ROS topics
- **ROS Node Commands**: List and info operations on ROS nodes  
- **ROS Parameter Commands**: List and get operations on ROS parameters

### Adding a Plugin

To add a new command plugin:

1. **Create the Plugin Service**

   Define your plugin service interface in `muto_msgs/srv`.

2. **Implement the Plugin Node**

   Create a ROS 2 node that provides the service interface.

3. **Update the Configuration**

   Add your plugin to the `commands` section in `muto.yaml`.

4. **Register the Plugin**

   Add the plugin executable to the package's setup.py.

For detailed instructions, refer to the [Adding a Plugin Guide](docs/adding-a-plugin.md).

## Message Handling

Agent processes several types of messages:

- **Gateway Messages**: Bidirectional communication with cloud digital twins
- **Command Messages**: Remote command execution requests and responses  
- **Composer Messages**: Software deployment and lifecycle management
- **Twin Messages**: Digital twin state synchronization

The message routing system ensures proper delivery to the appropriate Muto components based on message type and content.

## Eclipse Ditto/MQTT Integration

Agent provides secure MQTT connectivity for cloud integration:

- **Authentication**: Supports username/password and anonymous authentication
- **Topic Management**: Dynamic topic subscription based on vehicle namespace
- **Message Translation**: Converts between MQTT payloads and ROS 2 messages
- **Connection Management**: Automatic reconnection and error handling

Refer to the [Eclipse Ditto/MQTT Integration Guide](docs/mqtt-integration.md) for configuration details.

## Contributing

Contributions are welcome! Make sure you follow the [coding guidelines](./CODING_GUIDELINES.md) that were specified in the project wiki as much as you could.

## License

This project is licensed under the [EPL v2.0](LICENSE).

## Acknowledgements

- **ROS 2 Community**: For providing an excellent framework for robotic software development.
- **Eclipse Foundation**: For supporting the Eclipse Muto project.
- **Contributors**: Thanks to all the contributors who have helped improve Agent.
