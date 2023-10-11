cmake_minimum_required(VERSION 3.5)

add_executable(loop template_node.cpp)
target_include_directories(loop PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>  $<INSTALL_INTERFACE:include/>   ${rclcpp_INCLUDE_DIRS} ${rosgraph_msgs_INCLUDE_DIRS} ${LEMON_INCLUDE_DIRS})
target_link_libraries(loop data_flow_executor)
ament_target_dependencies(loop rclcpp std_msgs rosgraph_msgs)
add_dependencies(loop data_flow_executor) 



install(TARGETS
  loop
  DESTINATION lib/${PROJECT_NAME})