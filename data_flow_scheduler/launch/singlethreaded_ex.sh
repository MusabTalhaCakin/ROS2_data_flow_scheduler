#!/bin/bash

# Store the passed arguments in an array
arguments=("$@")

# Launch the ROS 2 node with the passed arguments
ros2 run autoware_reference_system autoware_singlethreaded_multi_process "${arguments[@]}"