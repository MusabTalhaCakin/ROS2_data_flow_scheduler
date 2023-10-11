enable_testing()
find_package(GTest REQUIRED)

add_executable(dfs_test test.cpp)
target_include_directories(dfs_test PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>  $<INSTALL_INTERFACE:include/>   ${rclcpp_INCLUDE_DIRS} ${rosgraph_msgs_INCLUDE_DIRS} ${LEMON_INCLUDE_DIRS})
target_link_libraries(dfs_test
 PRIVATE
  ${LEMON_LIBRARIES}
  data_flow_executor
  GTest::GTest
  )


add_test(NAME preloader_gtest COMMAND dfs_test)