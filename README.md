# Centralized Data Flow Scheduler

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)

The Centralized Data Flow Scheduler is a research project that aims to enhance the coordination capabilities of ROS2, a widely adopted robotic framework. By enabling cross-process coordination, the project addresses the need for scalable and flexible coordination of executables across multiple ROS2 nodes.

## Concept
The Centralized Data Flow Scheduler employs an innovative concept to enable efficient cross-process coordination within the ROS2 framework. At the core of this concept is a central control system responsible for managing and coordinating activities across multiple ROS2 nodes. This control system establishes Inter-Process Communication (IPC) socket connections, enabling the exchange of execution orders and seamless data flow between nodes.

## Installation

- **Developed and tested on** WSL2 (Ubuntu20.4)
- **Prerequisites** LEMON Graph Library [https://lemon.cs.elte.hu/trac/lemon]

## Build
Compile the ros2 package with:
```
$ colcon build
```
or
```
$ colcon build --package-select centralized_data_flow_scheduler autoware_reference_system reference_system reference_system_interface
```
## How to use
For the callbacks in the nodes to be scheduled a dfsexecutor has to be created:
```
#include "centralized_data_flow_scheduler/data_flow_executor.h" // namespace PAS_Interface
```
examples:
```
  DFS_Interface::DFSExecutor executor("node1",
    {{"time_", 0, 7470, node->return_timer_(), 0, "topic1"}});
```
```
  DFS_Interface::DFSExecutor executor("node2",
    {{"topic1", 1, 10470, node->return_subscription_(), 0, "topic2"}});
```
```
  DFS_Interface::DFSExecutor executor("node4",
    {{"topic1", 1, 1740, node->return_subscription_1(), 0, "topic4"},
    {"topic3", 1, 1830, node->return_subscription_2(), 1, "topic4"}});                                 
```
include Mutex & functionv and pass to the spin function. for a condition based publishing system.
```
std::mutex Mutex;
std::vector<std::function<void()>> functionv;
```
can be used like(not mandatory):
```
std::lock_guard<std::mutex> lock(Mutex);
functionv[0] = [this, message]()
{
  publisher_1->publish(message);
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
};
```
```
  activity.spin(functionv, Mutex); 
```

Before running the other nodes, execute the data_flow_scheduler node. It acts as the server for socket communication and waits for a specified number of client nodes to connect. The number passed as an argument when running the data_flow_scheduler is the desired number of client nodes to connect with.
```
$ ros2 run centralized_data_flow_scheduler data_flow_scheduler 24
```
## Changes in autoware_reference_system
to test the data_flow_scheduler with the reference system nodes the shared pointer to the activities has to be accesssible. following code has been added for each pointer:

```
rclcpp::SubscriptionBase::SharedPtr return_subscription_(){
  return subscription_;
}
```

## Test
To test the conzept launch the files:

```
ros2 launch centralized_data_flow_scheduler demo_graph_launch.py
```
to test it with the autoware reference system run:
```
ros2 launch centralized_data_flow_scheduler autoware_reference_system_launch.py
```

## License

The Centralized Data Flow Scheduler is released under the Apache 2.0 License. For more details, please refer to the [LICENSE](LICENSE) file.
