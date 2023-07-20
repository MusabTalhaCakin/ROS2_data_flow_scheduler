/*******************************************************************************
 * Copyright (C) 2023 TTTech Auto AG. All rights reserved                      *
 * Operngasse 17-21, 1040 Vienna, Austria. office(at)tttech-auto.com           *
 ******************************************************************************/

#include "rclcpp/rclcpp.hpp"
#include "reference_system/system/type/rclcpp_system.hpp"
#include "autoware_reference_system/autoware_system_builder.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  int nt = (argc > 1) ? atoi(argv[1]) : 1;
  std::vector<std::string> arguments;
  for (int i = 3; i < argc; ++i)
  {
    arguments.push_back(argv[i]);
  }

  SampleManagementSettings::get().set_hot_path(
      {"FrontLidarDriver",
       "RearLidarDriver",
       "PointsTransformerFront",
       "PointsTransformerRear",
       "PointCloudFusion",
       "RayGroundFilter",
       "EuclideanClusterDetector",
       "ObjectCollisionEstimator"},
      {"FrontLidarDriver", "RearLidarDriver"},
      "ObjectCollisionEstimator");

  switch (nt)
  {
  case 1: // SENSOR NODE
  {
    std::chrono::nanoseconds time_t_value(std::stoi(argv[2]));
    auto node = std::make_shared<nodes::rclcpp_system::Sensor>(
        nodes::SensorSettings{
            .node_name = arguments[0],
            .topic_name = arguments[1],
            .cycle_time = time_t_value});
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    break;
  }
  case 2: // TRANSFORM NODE
  {
    uint64_t time_t_value(std::stoi(argv[2]));
    auto node = std::make_shared<nodes::rclcpp_system::Transform>(
        nodes::TransformSettings{
            .node_name = arguments[0],
            .input_topic = arguments[1],
            .output_topic = arguments[2],
            .number_crunch_limit = time_t_value});
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    break;
  }
  case 3: // FUSION NODE
  {
    uint64_t time_t_value(std::stoi(argv[2]));
    auto node = std::make_shared<nodes::rclcpp_system::Fusion>(
        nodes::FusionSettings{
            .node_name = arguments[0],
            .input_0 = arguments[1],
            .input_1 = arguments[2],
            .output_topic = arguments[3],
            .number_crunch_limit = time_t_value});
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    break;
  }
  case 4: // CYCLIC NODE
  {
    uint64_t time_t_value(std::stoi(argv[2]));
    auto node = std::make_shared<nodes::rclcpp_system::Cyclic>(
        nodes::CyclicSettings{
            .node_name = arguments[0],
            .inputs = {arguments[1], arguments[2],
                       arguments[3], arguments[4],
                       arguments[5], arguments[6]},
            .output_topic = arguments[0],
            .number_crunch_limit = time_t_value,
            .cycle_time = std::chrono::milliseconds(100)});
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    break;
  }
  case 5: // INTERSECTION NODE
  {
    uint64_t time_t_value(std::stoi(argv[2]));
    auto node = std::make_shared<nodes::rclcpp_system::Intersection>(
        nodes::IntersectionSettings{
            .node_name = arguments[0],
            .connections = {
                {.input_topic = arguments[1],
                 .output_topic = arguments[2],
                 .number_crunch_limit = time_t_value},
                {.input_topic = arguments[3],
                 .output_topic = arguments[4],
                 .number_crunch_limit = time_t_value}}});
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    break;
  }
  case 6: // COMMAND NODE
  {
    uint64_t time_t_value(std::stoi(argv[2]));
    auto node = std::make_shared<nodes::rclcpp_system::Command>(
        nodes::CommandSettings{
            .node_name = arguments[0],
            .input_topic = arguments[1]});

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    break;
  }
  default:
    break;
  }

  rclcpp::shutdown();
  return 0;
}
