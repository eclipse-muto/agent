#
#  Copyright (c) 2023 Composiv.ai
#
# All rights reserved. This program and the accompanying materials
# are made available under the terms of the Eclipse Public License v2.0
# and Eclipse Distribution License v1.0 which accompany this distribution.
#
# Licensed under the  Eclipse Public License v2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# The Eclipse Public License is available at
#    http://www.eclipse.org/legal/epl-v20.html
# and the Eclipse Distribution License is available at
#   http://www.eclipse.org/org/documents/edl-v10.php.
#
# Contributors:
#    Composiv.ai - initial API and implementation
#
#

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

pkg_name = "agent"
output = "screen"

def generate_launch_description():

    # Files
    file_config = os.path.join(get_package_share_directory(pkg_name), "config", "agent.yaml")


    # Nodes
    node_agent = Node(
        name="muto_agent",
        package="agent",
        executable="muto_agent",
        output=output,
        parameters=[file_config]
    )

    node_mqtt_gateway = Node(
        name="mqtt_gateway",
        package="agent",
        executable="mqtt",
        output=output,
        parameters=[file_config]
    )

    node_commands = Node(
        name="commands_plugin",
        package="agent",
        executable="commands",
        output=output,
        parameters=[file_config]
    )


    # Launch Description Object
    ld = LaunchDescription()

    ld.add_action(node_agent)
    ld.add_action(node_mqtt_gateway)
    ld.add_action(node_commands)

    return ld