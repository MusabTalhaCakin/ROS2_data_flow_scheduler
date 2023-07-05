cmake_minimum_required(VERSION 3.5)
project(centralized_data_flow_scheduler)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

set(CMAKE_CXX_STANDARD 17)
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(LEMON REQUIRED PATHS ${CMAKE_CURRENT_SOURCE_DIR}"/lib/lemon/share/lemon/cmake")

#include directories
include_directories(${rclcpp_INCLUDE_DIRS})
include_directories(${rosgraph_msgs_INCLUDE_DIRS})
include_directories(${LEMON_INCLUDE_DIRS})
# Add the Lemon libraries to the link path
link_directories(${LEMON_LIBRARY_DIRS})

include_directories(include)

#Add Librarie
add_library(data_flow_executor src/data_flow_executor.cpp src/ipc_client.cpp src/callback_handler.cpp)
target_link_libraries(data_flow_executor)
ament_target_dependencies(data_flow_executor rclcpp rosidl_runtime_cpp)
ament_export_targets(data_flow_executor HAS_LIBRARY_TARGET)

#Add Nodes
add_executable(data_flow_scheduler src/data_flow_scheduler.cpp src/graphcreator.cpp src/data_flow_sequencer.cpp src/ipc_server.cpp src/data_flow_hashT.cpp)
target_link_libraries(data_flow_scheduler ${LEMON_LIBRARIES})
ament_target_dependencies(data_flow_scheduler rclcpp std_msgs rosgraph_msgs)

add_executable(template_node src/test/template_node.cpp)
target_link_libraries(template_node data_flow_executor)
ament_target_dependencies(template_node rclcpp std_msgs rosgraph_msgs)
add_dependencies(template_node data_flow_executor) 

install(
  DIRECTORY include/centralized_data_flow_scheduler
  DESTINATION include
)
 
install(
  TARGETS data_flow_executor 
  EXPORT data_flow_executor 
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS
  data_flow_scheduler
  template_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
ament_package()