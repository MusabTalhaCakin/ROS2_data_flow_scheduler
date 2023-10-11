/*******************************************************************************
 * Copyright (C) 2023 TTTech Auto AG. All rights reserved                      *
 * Operngasse 17-21, 1040 Vienna, Austria. office(at)tttech-auto.com           *
 ******************************************************************************/

#include "autoware_reference_system/autoware_system_builder.hpp"
#include "data_flow_scheduler/data_flow_executor.h"
#include "rclcpp/rclcpp.hpp"
#include "reference_system/system/type/rclcpp_system.hpp"

int main(int argc, char *argv[])
{
  // Testing the Autoware reference system with the data flow scheduler on
  // multiple processes run the
  // "/data_flow_scheduler/launch/autoware_reference_system_launch.py"

  // set_benchmark_mode(true);
  rclcpp::init(argc, argv);

  int nt = (argc > 1) ? atoi(argv[1]) : 1;
  int time_(std::stoi(argv[2]));

  std::vector<std::string> arguments;
  for (int i = 4; i < argc; ++i)
  {
    arguments.push_back(argv[i]);
  }

  SampleManagementSettings::get().set_hot_path(
      {"FrontLidarDriver", "RearLidarDriver", "PointsTransformerFront",
       "PointsTransformerRear", "PointCloudFusion", "RayGroundFilter",
       "EuclideanClusterDetector", "ObjectCollisionEstimator"},
      {"FrontLidarDriver", "RearLidarDriver"}, "ObjectCollisionEstimator");

  switch (nt)
  {
  case 1: // SENSOR NODE
  {
    std::chrono::microseconds time_t_value(std::stoi(argv[3]));
    auto node = std::make_shared<nodes::rclcpp_system::Sensor>(
        nodes::SensorSettings{.node_name = arguments[0],
                              .topic_name = arguments[1],
                              .cycle_time = time_t_value});

    DFSched::DFSExecutor DFSExecutor(
        arguments[0],
        {{arguments[0], 0, time_, node->return_timer_(), 0, arguments[0], DFSched::TimeSupervision::Time}});

    DFSExecutor.spin();
    break;
  }
  case 2: // TRANSFORM NODE
  {
    uint64_t time_t_value(std::stoi(argv[3]));
    auto node = std::make_shared<nodes::rclcpp_system::Transform>(
        nodes::TransformSettings{.node_name = arguments[0],
                                 .input_topic = arguments[1],
                                 .output_topic = arguments[2],
                                 .number_crunch_limit = time_t_value});
    DFSched::DFSExecutor DFSExecutor(
        arguments[0], {{arguments[1], 1, time_, node->return_subscription_(), 0,
                        arguments[2], DFSched::TimeSupervision::Time}});

    std::vector<std::function<void()>> functionVector;
    std::mutex vectorMutex;
    DFSExecutor.spin();
    break;
  }
  case 3: // FUSION NODE
  {
    uint64_t time_t_value(std::stoi(argv[3]));
    auto node = std::make_shared<nodes::rclcpp_system::Fusion>(
        nodes::FusionSettings{.node_name = arguments[0],
                              .input_0 = arguments[1],
                              .input_1 = arguments[2],
                              .output_topic = arguments[3],
                              .number_crunch_limit = time_t_value});

    DFSched::CallbackInfoVector vinfo(1);
    std::vector<std::string> subscribe_topics{arguments[1], arguments[2]};

    std::vector<std::string> publish_topics{arguments[3]};

    vinfo[0].subs = subscribe_topics;
    // Type of node: n:1 node
    //(zayas) create enum for this magic values
    vinfo[0].type = 3;
    vinfo[0].runtime = time_t_value;

    //  here we define the callback that is going to be called
    //  when precedesors are done
    vinfo[0].callback_ptr = [&node]()
    { node->another_callback(); };
    vinfo[0].id = 0;
    vinfo[0].pubs = publish_topics;

    DFSched::DFSExecutor DFSExecutor(arguments[0], vinfo);

    std::vector<std::function<void()>> functionVector;
    std::mutex vectorMutex;
    DFSExecutor.spin();
    break;
  }
  case 4: // CYCLIC NODE
  {
    uint64_t time_t_value(std::stoi(argv[3]));
    auto node =
        std::make_shared<nodes::rclcpp_system::Cyclic>(nodes::CyclicSettings{
            .node_name = arguments[0],
            .inputs = {arguments[1], arguments[2], arguments[3], arguments[4],
                       arguments[5], arguments[6]},
            .output_topic = arguments[0],
            .number_crunch_limit = time_t_value,
            .cycle_time = std::chrono::milliseconds(0)});

    DFSched::CallbackInfoVector vinfo(1);
    std::vector<std::string> subscribe_topics{arguments[1], arguments[2],
                                              arguments[3], arguments[4],
                                              arguments[5], arguments[6]};

    std::vector<std::string> publish_topics{arguments[0]};

    vinfo[0].subs = subscribe_topics;
    // Type of node: n:1 node
    //(zayas) create enum for this magic values
    vinfo[0].type = 3;
    vinfo[0].runtime = time_t_value;

    //  here we define the callback that is going to be called
    //  when precedesors are done
    vinfo[0].callback_ptr = [&node]()
    { node->timer_callback(); };
    vinfo[0].id = 0;
    vinfo[0].pubs = publish_topics;

    std::vector<std::function<void()>> functionVector;
    std::mutex vectorMutex;
    DFSched::DFSExecutor DFSExecutor(arguments[0], vinfo);
    DFSExecutor.spin();
    break;
  }
  case 5: // INTERSECTION NODE
  {
    uint64_t time_t_value(std::stoi(argv[3]));
    auto node = std::make_shared<nodes::rclcpp_system::Intersection>(
        nodes::IntersectionSettings{
            .node_name = arguments[0],
            .connections = {{.input_topic = arguments[1],
                             .output_topic = arguments[2],
                             .number_crunch_limit = time_t_value},
                            {.input_topic = arguments[3],
                             .output_topic = arguments[4],
                             .number_crunch_limit = time_t_value}}});

    std::vector<std::function<void()>> functionVector;
    std::mutex vectorMutex;

    DFSched::CallbackInfoVector vinfo(1);
    std::vector<std::string> subscribe_topics{arguments[1], arguments[3]};

    std::vector<std::string> publish_topics{arguments[2], arguments[4]};

    vinfo[0].subs = subscribe_topics;
    // Type of node: n:1 node
    //(zayas) create enum for this magic values
    vinfo[0].type = 3;
    vinfo[0].runtime = time_t_value;
    vinfo[0].pubs = publish_topics;
    //  here we define the callback that is going to be called
    //  when precedesors are done
    vinfo[0].callback_ptr = [&node]()
    { node->another_callback(); };
    vinfo[0].id = 0;
    DFSched::DFSExecutor DFSExecutor(arguments[0], vinfo);
    DFSExecutor.spin();

    break;
  }
  case 6: // COMMAND NODE
  {
    auto node =
        std::make_shared<nodes::rclcpp_system::Command>(nodes::CommandSettings{
            .node_name = arguments[0], .input_topic = arguments[1]});
    rclcpp::SubscriptionBase::SharedPtr sub = node->return_subscription_();

    DFSched::DFSExecutor DFSExecutor(
        arguments[0], {{arguments[1], 1, time_, sub, 0, "", DFSched::TimeSupervision::Time}});

    DFSExecutor.spin();
    break;
  }
  case 8: // CYCLIC FUSION NODE
  {
    uint64_t time_t_value(std::stoi(argv[3]));
    auto node = std::make_shared<nodes::rclcpp_system::Cyclic>(
        nodes::CyclicSettings{.node_name = arguments[0],
                              .inputs = {arguments[1], arguments[2]},
                              .output_topic = arguments[0],
                              .number_crunch_limit = time_t_value,
                              .cycle_time = std::chrono::milliseconds(0)});

    DFSched::CallbackInfoVector vinfo(1);
    std::vector<std::string> subscribe_topics{arguments[1], arguments[2]};

    std::vector<std::string> publish_topics{arguments[0]};

    vinfo[0].subs = subscribe_topics;
    // Type of node: n:1 node
    //(zayas) create enum for this magic values
    vinfo[0].type = 3;
    vinfo[0].runtime = time_t_value;
    vinfo[0].pubs = publish_topics;
    //  here we define the callback that is going to be called
    //  when precedesors are done
    vinfo[0].callback_ptr = [&node]()
    { node->timer_callback(); };
    vinfo[0].id = 0;

    DFSched::DFSExecutor DFSExecutor(arguments[0], vinfo);
    DFSExecutor.spin();
    break;
  }
  default:
    break;
  }

  rclcpp::shutdown();
  return 0;
}
