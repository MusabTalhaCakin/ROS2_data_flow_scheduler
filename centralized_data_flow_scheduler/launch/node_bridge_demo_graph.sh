#!/bin/bash

# Store the passed arguments in an array
arguments=("$@")

# Launch the ROS 2 node with the passed arguments
ros2 run centralized_data_flow_scheduler template_node "${arguments[@]}"