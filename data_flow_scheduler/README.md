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

## License
This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.
