# Data Flow Scheduler

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)

# Note
This is a modified version of the original **Data Flow Scheduler** with some additional features: 
- Configurable activity timeout supervision based on thread CPU time or system time
- `n:1` subscription improved
- New examples




# ROS2 Data Flow Scheduler

This project was conducted as a master's thesis in collaboration with [TTTech Auto AG](https://www.tttech.com). The research project aims to enhance the coordination capabilities of ROS2, a widely adopted robotic framework. By enabling cross-process coordination, the project addresses the need for scalable and flexible coordination of executables across multiple ROS2 nodes

## Concept
The Data Flow Scheduler employs an innovative concept to enable efficient cross-process coordination within the ROS2 framework. At its core, this concept utilizes a control system responsible for managing and coordinating the execution of callbacks across multiple ROS2 nodes. The system establishes Inter-Process socket Communication (IPC), facilitating the exchange of execution orders and ensuring data flow between nodes.

## Installation

- **Developed and tested on:**
  - WSL2 (Ubuntu 20.04)
  - Ubuntu 22.04

- **Tested on:**
  - Raspberry Pi 4 B 8GB RAM (Ubuntu Server 22.04)

- **Prerequisites:**
  - LEMON Graph Library [https://lemon.cs.elte.hu/trac/lemon]

Please refer to the [LEMON Graph Library website](https://lemon.cs.elte.hu/trac/lemon/wiki/InstallLinux) for installation guidelines and instructions.

## Build
To compile the ROS2 package, use `colcon build`:
```
colcon build
```

Alternatively, if you only want to build specific packages, use the `--packages-select` option:
```
colcon build --packages-select data_flow_scheduler autoware_reference_system reference_system reference_system_interface
```

This will build the following packages: data_flow_scheduler, autoware_reference_system, reference_system, and reference_system_interface.

## How to use
See [examples](data_flow_scheduler/examples) folder for some examples on how to deploy a node network and the callbacks.




## Configuration
Take a look to [data_flow_types.h](data_flow_scheduler/include/data_flow_scheduler/data_flow_types.h) to configure the system.

Some available parameters are:
- Number of parallel activities to dispatch
- Threads priority
- Scheduling policies




# Test with custom network
This test deploys a node network to test the latency of the system.

```
ros2 launch data_flow_scheduler demo_graph_launch.py
```

# Test with Autoware Reference System
This test runs the Autoware Reference System where each node runs in a separate process.

```
ros2 launch data_flow_scheduler autoware_reference_system_launch.py
```

## Changes in autoware_reference_system
To test the `data_flow_scheduler` with the reference system nodes the shared pointer to the activities has to be accesssible. Following code has been added for each pointer:

```
rclcpp::SubscriptionBase::SharedPtr return_subscription_(){
  return subscription_;
}
```

Also [fusion node](reference-system/reference_system/include/reference_system/nodes/rclcpp/fusion.hpp) has been modified to provide subscription for more than 1 topic.

Check the [intersection node](reference-system/autoware_reference_system/src/ros2/executor/reference_system_dfsexecutor_interface_node.cpp) and the  [examples](data_flow_scheduler/examples) folder.

  

# TODO list, future work and limitations

- The `DFSched::CallbackInfo`, `DFSched::NodeInfo`, `DFSched::TopicInfo` and `DFSched::TimerInfo` needs to be unified.
- Data structures that are holding the graph status can be improved for better perfomance.
- More decision algorithms can be implemented.
- Functions and methods like `CallbackHandler::run_callback()` needs to be rewritten in a better way
- Dynamic memory during runtime must be avoided
- Thread creation during rumtime must be removed  




## Tricks

It is possible to disable the longest path in [graphcreator.cpp:188](data_flow_scheduler/src/graphcreator.cpp) just commenting out the call to `compute_longestpath();`

This enables a pseudo random choice of the node.



# License
The Data Flow Scheduler is released under the Apache 2.0 License. For more details, please refer to the [LICENSE](LICENSE) file.
