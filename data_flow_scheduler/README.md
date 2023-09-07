# Data Flow Scheduler

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)

## Introduction
The Data Flow Scheduler is a ROS2 package that provides functionalities to manage the execution of callbacks in a ROS2 node network. It allows you to control the scheduling behavior and set various parameters to fine-tune the execution environment.

## Parameters
Parameters are initialized and set in the `data_flow_types.h` file. The following are the parameters with their default values:

- **CORES**: 1 (Max. number of simultaneously executable callbacks)
- **SET_THREAD_PRIORITY**: false (set to true to set thread priorities, requires root permissions)
- **CREATE_LGF**: false (set to true to create an lgf file of the ROS2 node network graph)
- **VERBOSE**: false (set to true for Data Flow Scheduler state output, will decrease scheduling speed)
- **THROW_ITERATION_TIMEOUT**: false (Throw iteration if Timeout occurs)
- **THROW_ITERATION_EXECUTION_FAIL**: false (Throw iteration if the execution of a callback fails)
- **ITERATION**: 100000 (Set periodic execution, 0 for no periodic consideration)
- **RUNTIME**: 0 (in seconds, 0 for infinite)


Additional configurations within the launch files: 
- **Taskset**: specify the core on which to run the process for each node, **['taskset', '-c', '3']**.
- **Timeout**: set the timeout Value for each callback within the launch files, **[str(5000)]**.
- **Pseudo workload**: Set the pseudo workload of the callbacks, **[str(1024)]** (see https://github.com/MusabTalhaCakin/ROS2_data_flow_scheduler/tree/main/reference-system/autoware_reference_system, [Configure Processing Time]).

Example:
```
t_node1 = ExecuteProcess(
    cmd=['taskset', '-c', '3', bash_file_path, '2', str(5000), str(1024), 'PointsTransformerFront',
    'FrontLidarDriver', 'PointsTransformerFront'],
    output='screen'
)
``````

## License
This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.
