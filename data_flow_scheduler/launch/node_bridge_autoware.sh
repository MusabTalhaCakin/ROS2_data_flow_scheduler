#!/bin/bash

# Store the passed arguments in an array
arguments=("$@")

# Launch the ROS 2 node with the passed arguments
ros2 run autoware_reference_system reference_system_dfsexecutor_interface_node "${arguments[@]}"