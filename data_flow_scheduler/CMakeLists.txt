cmake_minimum_required(VERSION 3.5)
project(data_flow_scheduler)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

set(CMAKE_CXX_STANDARD 17)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -ggdb -O2)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(LEMON REQUIRED PATHS ${CMAKE_CURRENT_SOURCE_DIR}"/lib/lemon/share/lemon/cmake")

include_directories(include)

#Add Library
add_library(data_flow_executor 
src/data_flow_executor.cpp 
src/ipc_client.cpp 
src/callback_handler.cpp
include/data_flow_scheduler/data_flow_types.h
include/data_flow_scheduler/data_flow_executor.h
include/data_flow_scheduler/ipc_client.h
)
target_link_libraries(data_flow_executor)
ament_target_dependencies(data_flow_executor rclcpp rosidl_runtime_cpp)
ament_export_targets(data_flow_executor HAS_LIBRARY_TARGET)
target_include_directories(data_flow_executor PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>  $<INSTALL_INTERFACE:include/>   ${rclcpp_INCLUDE_DIRS} ${rosgraph_msgs_INCLUDE_DIRS} ${LEMON_INCLUDE_DIRS})


#Add Nodes
add_executable(data_flow_scheduler 
src/data_flow_scheduler.cpp 
src/graphcreator.cpp 
src/data_flow_sequencer.cpp 
src/ipc_server.cpp 
src/data_flow_hashT.cpp
include/data_flow_scheduler/data_flow_types.h
include/data_flow_scheduler/data_flow_sequencer.h
include/data_flow_scheduler/graphcreator.h
include/data_flow_scheduler/igraphcreator.h
include/data_flow_scheduler/ipc_server.h
include/data_flow_scheduler/data_flow_hashT.h

)
target_link_libraries(data_flow_scheduler ${LEMON_LIBRARIES})
ament_target_dependencies(data_flow_scheduler rclcpp std_msgs rosgraph_msgs)
target_include_directories(data_flow_scheduler PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>  $<INSTALL_INTERFACE:include/>   ${rclcpp_INCLUDE_DIRS} ${rosgraph_msgs_INCLUDE_DIRS} ${LEMON_INCLUDE_DIRS})

install(
  DIRECTORY include/data_flow_scheduler
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
  DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
ament_package()

add_subdirectory(examples)

# add_subdirectory(test)