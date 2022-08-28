#
#  Copyright (c) 2022 Composiv.ai, Eteration A.S. and others
#
# All rights reserved. This program and the accompanying materials
# are made available under the terms of the Eclipse Public License v2.0
# and Eclipse Distribution License v1.0 which accompany this distribution.
#
# The Eclipse Public License is available at
#    http://www.eclipse.org/legal/epl-v10.html
# and the Eclipse Distribution License is available at
#   http://www.eclipse.org/org/documents/edl-v10.php.
#
# Contributors:
#    Composiv.ai, Eteration A.S. - initial API and implementation
#
#
cmake_minimum_required(VERSION 3.0.2)
project(muto_agent)

find_package(catkin REQUIRED COMPONENTS
ackermann_msgs
geometry_msgs
nav_msgs
roscpp
rospy
roslaunch
sensor_msgs
std_msgs
muto_msgs
muto_core
)

catkin_python_setup()

catkin_package()
