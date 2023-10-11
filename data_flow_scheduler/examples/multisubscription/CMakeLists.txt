cmake_minimum_required(VERSION 3.5)
project(data_flow_scheduler)

add_executable(multisub multisub.cpp)
target_include_directories(multisub PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>  $<INSTALL_INTERFACE:include/>   ${rclcpp_INCLUDE_DIRS} ${rosgraph_msgs_INCLUDE_DIRS} ${LEMON_INCLUDE_DIRS})
target_link_libraries(multisub data_flow_executor)
ament_target_dependencies(multisub rclcpp std_msgs rosgraph_msgs)
add_dependencies(multisub data_flow_executor) 

add_executable(template_node template_node.cpp)
target_include_directories(template_node PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>  $<INSTALL_INTERFACE:include/>   ${rclcpp_INCLUDE_DIRS} ${rosgraph_msgs_INCLUDE_DIRS} ${LEMON_INCLUDE_DIRS})
target_link_libraries(template_node data_flow_executor)
ament_target_dependencies(template_node rclcpp std_msgs rosgraph_msgs)
add_dependencies(template_node data_flow_executor) 

install(TARGETS
  multisub
  template_node
  DESTINATION lib/${PROJECT_NAME})