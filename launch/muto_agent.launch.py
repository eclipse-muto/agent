import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch import LaunchDescription

pkg_name = "muto_agent"
output = "screen"


def generate_launch_description():
    # Arguments
    # Use this file's location to get config files
    config_dir = os.path.join(get_package_share_directory(pkg_name), "config")

    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "enable_symphony",
            default_value="true",
            description="Enable Symphony MQTT provider",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="INFO",
            description="Logging level for all nodes",
            choices=["DEBUG", "INFO", "WARN", "ERROR"],
        ),
        DeclareLaunchArgument(
            "muto_config_file",
            default_value=os.path.join(config_dir, "muto.yaml"),
            description="Path to global Muto configuration file",
        ),
        DeclareLaunchArgument("muto_namespace", default_value="muto"),
        DeclareLaunchArgument(
            "vehicle_namespace",
            default_value="org.eclipse.muto.sandbox",
            description="Vehicle ID namespace",
        ),
        DeclareLaunchArgument("vehicle_name", description="Vehicle ID"),
    ]

    # Configuration parameters
    enable_symphony = LaunchConfiguration("enable_symphony")
    log_level = LaunchConfiguration("log_level")
    muto_config_file = LaunchConfiguration("muto_config_file")
    vehicle_namespace = LaunchConfiguration("vehicle_namespace")
    vehicle_name = LaunchConfiguration("vehicle_name")
    muto_namespace = LaunchConfiguration("muto_namespace")

    # Agent
    node_agent = Node(
        namespace=muto_namespace,
        name="agent",
        package="muto_agent",
        executable="muto_agent",
        output="screen",
        parameters=[
            muto_config_file,
            {"namespace": vehicle_namespace},
            {"name": vehicle_name},
        ],
    )

    node_mqtt_gateway = Node(
        namespace=muto_namespace,
        name="gateway",
        package="muto_agent",
        executable="mqtt",
        output="screen",
        parameters=[
            muto_config_file,
            {"namespace": vehicle_namespace},
            {"name": vehicle_name},
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    node_commands = Node(
        namespace=muto_namespace,
        name="commands_plugin",
        package="muto_agent",
        executable="commands",
        output="screen",
        parameters=[
            muto_config_file,
            {"namespace": vehicle_namespace},
            {"name": vehicle_name},
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    # Symphony Provider (conditional)

    symphony_provider = Node(
        namespace=muto_namespace,
        package="muto_agent",
        executable="symphony_provider",
        name="muto_symphony_provider",
        output="screen",
        condition=IfCondition(enable_symphony),
        parameters=[
            muto_config_file,
            {"name": vehicle_name},
            {"namespace": vehicle_namespace},
            {"symphony_target_name": vehicle_name},
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    # Launch Description Object
    ld = LaunchDescription()

    # add all declared arguments
    for arg in declared_arguments:
        ld.add_action(arg)

    # add all nodes
    ld.add_action(node_agent)
    ld.add_action(node_mqtt_gateway)
    ld.add_action(node_commands)
    ld.add_action(symphony_provider)

    return ld
