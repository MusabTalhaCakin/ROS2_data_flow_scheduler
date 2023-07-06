/*******************************************************************************
 * Copyright (C) 2023 TTTech Auto AG. All rights reserved                      *
 * Operngasse 17-21, 1040 Vienna, Austria. office(at)tttech-auto.com           *
 ******************************************************************************/

#include "rclcpp/rclcpp.hpp"
#include "reference_system/system/type/rclcpp_system.hpp"
#include "autoware_reference_system/autoware_system_builder.hpp"
#include "autoware_reference_system/system/timing/benchmark.hpp"
#include "autoware_reference_system/system/timing/default.hpp"
#include "data_flow_scheduler/data_flow_executor.h"

void sensor_node_DFSExecutor(
    const std::vector<std::shared_ptr<rclcpp::Node>> &nodes,
    const std::string &name_timer,
    const std::string &name_pub,
    int runtime)
{

    rclcpp::TimerBase::SharedPtr timer =
        std::dynamic_pointer_cast<nodes::rclcpp_system::Sensor>(get_node(name_pub, nodes))->return_timer_();

    DFS_Interface::DFSExecutor DFSExecutor(name_pub,
                                           {{name_timer, 0, runtime, timer, 0, name_pub}});

    std::vector<std::function<void()>> functionVector;
    std::mutex mutex_;
    DFSExecutor.spin(functionVector, mutex_);
}

void transform_node_DFSExecutor(
    const std::vector<std::shared_ptr<rclcpp::Node>> &nodes,
    const std::string &name_sub,
    const std::string &name_pub,
    int runtime)
{

    rclcpp::SubscriptionBase::SharedPtr sub =
        std::dynamic_pointer_cast<nodes::rclcpp_system::Transform>(get_node(name_pub, nodes))->return_subscription_();

    DFS_Interface::DFSExecutor DFSExecutor(name_pub,
                                           {{name_sub, 1, runtime, sub, 0, name_pub}});

    std::vector<std::function<void()>> functionVector;
    std::mutex mutex_;
    DFSExecutor.spin(functionVector, mutex_);
}

void fusion_node_DFSExecutor(
    const std::vector<std::shared_ptr<rclcpp::Node>> &nodes,
    const std::string &name_sub1,
    const std::string &name_sub2,
    const std::string &name_pub,
    int runtime)
{

    rclcpp::SubscriptionBase::SharedPtr sub1 =
        std::dynamic_pointer_cast<nodes::rclcpp_system::Fusion>(get_node(name_pub, nodes))->return_subscription_1();

    rclcpp::SubscriptionBase::SharedPtr sub2 =
        std::dynamic_pointer_cast<nodes::rclcpp_system::Fusion>(get_node(name_pub, nodes))->return_subscription_2();

    DFS_Interface::DFSExecutor DFSExecutor(name_pub,
                                           {{name_sub1, 1, runtime, sub1, 0, name_pub}, {name_sub2, 1, runtime, sub2, 1, name_pub}});

    std::vector<std::function<void()>> functionVector;
    std::mutex mutex_;
    DFSExecutor.spin(functionVector, mutex_);
}

void cyclic_node_DFSExecutor(
    const std::vector<std::shared_ptr<rclcpp::Node>> &nodes,
    const std::string &name_timer,
    const std::string &name_sub1,
    const std::string &name_sub2,
    const std::string &name_sub3,
    const std::string &name_sub4,
    const std::string &name_sub5,
    const std::string &name_sub6,
    const std::string &name_pub,
    int runtime)
{

    rclcpp::TimerBase::SharedPtr timer =
        std::dynamic_pointer_cast<nodes::rclcpp_system::Cyclic>(get_node(name_pub, nodes))->return_timer_();

    rclcpp::SubscriptionBase::SharedPtr sub1 =
        std::dynamic_pointer_cast<nodes::rclcpp_system::Cyclic>(get_node(name_pub, nodes))->return_subscription_1();

    rclcpp::SubscriptionBase::SharedPtr sub2 =
        std::dynamic_pointer_cast<nodes::rclcpp_system::Cyclic>(get_node(name_pub, nodes))->return_subscription_2();

    rclcpp::SubscriptionBase::SharedPtr sub3 =
        std::dynamic_pointer_cast<nodes::rclcpp_system::Cyclic>(get_node(name_pub, nodes))->return_subscription_3();

    rclcpp::SubscriptionBase::SharedPtr sub4 =
        std::dynamic_pointer_cast<nodes::rclcpp_system::Cyclic>(get_node(name_pub, nodes))->return_subscription_4();

    rclcpp::SubscriptionBase::SharedPtr sub5 =
        std::dynamic_pointer_cast<nodes::rclcpp_system::Cyclic>(get_node(name_pub, nodes))->return_subscription_5();

    rclcpp::SubscriptionBase::SharedPtr sub6 =
        std::dynamic_pointer_cast<nodes::rclcpp_system::Cyclic>(get_node(name_pub, nodes))->return_subscription_6();

    DFS_Interface::DFSExecutor DFSExecutor(name_pub,
                                           {{name_timer, 0, runtime, timer, 0, name_pub}},
                                           {{name_sub1, 1, runtime, sub1, 0, name_timer}, {name_sub2, 1, runtime, sub2, 1, name_timer}, {name_sub3, 1, runtime, sub3, 2, name_timer}, {name_sub4, 1, runtime, sub4, 3, name_timer}, {name_sub5, 1, runtime, sub5, 4, name_timer}, {name_sub6, 1, runtime, sub6, 5, name_timer}});

    std::vector<std::function<void()>> functionVector;
    std::mutex mutex_;
    DFSExecutor.spin(functionVector, mutex_);
}

void intersection_node_DFSExecutor(
    const std::vector<std::shared_ptr<rclcpp::Node>> &nodes,
    const std::string node_name,
    const std::string &name_sub1,
    const std::string &name_sub2,
    const std::string &name_pub1,
    const std::string &name_pub2,
    int runtime)
{

    rclcpp::SubscriptionBase::SharedPtr sub1 =
        std::dynamic_pointer_cast<nodes::rclcpp_system::Intersection>(get_node(name_pub1, nodes))->return_subscription_1();

    rclcpp::SubscriptionBase::SharedPtr sub2 =
        std::dynamic_pointer_cast<nodes::rclcpp_system::Intersection>(get_node(name_pub1, nodes))->return_subscription_2();

    DFS_Interface::DFSExecutor DFSExecutor(node_name,
                                           {{name_sub1, 1, runtime, sub1, 0, name_pub1}, {name_sub2, 1, runtime, sub2, 1, name_pub2}});

    std::vector<std::function<void()>> functionVector;
    std::mutex mutex_;
    DFSExecutor.spin(functionVector, mutex_);
}

void command_node_DFSExecutor(
    const std::vector<std::shared_ptr<rclcpp::Node>> &nodes,
    const std::string &name_sub,
    const std::string &name_,
    int runtime)
{

    rclcpp::SubscriptionBase::SharedPtr sub =
        std::dynamic_pointer_cast<nodes::rclcpp_system::Command>(get_node(name_, nodes))->return_subscription_();

    DFS_Interface::DFSExecutor DFSExecutor(name_,
                                           {{name_sub, 1, runtime, sub, 0}});

    std::vector<std::function<void()>> functionVector;
    std::mutex mutex_;
    DFSExecutor.spin(functionVector, mutex_);
}

int main(int argc, char *argv[])
{
    // Testing the data flow scheduler on the Autoware reference system.

    rclcpp::init(argc, argv);
    using TimeConfig = nodes::timing::Default;
    auto nodes = create_autoware_nodes<RclcppSystem, TimeConfig>();
    std::vector<std::thread> threads;

    int timeout_timer = 100000, timeout_topic = 100000;

    // SENSOR NODES
    threads.emplace_back([&nodes, timeout_timer]()
                         { sensor_node_DFSExecutor(nodes, "timer_1", "FrontLidarDriver", timeout_timer); });
    usleep(1000);

    threads.emplace_back([&nodes, timeout_timer]()
                         { sensor_node_DFSExecutor(nodes, "timer_2", "RearLidarDriver", timeout_timer); });
    usleep(1000);

    threads.emplace_back([&nodes, timeout_timer]()
                         { sensor_node_DFSExecutor(nodes, "timer_3", "PointCloudMap", timeout_timer); });
    usleep(1000);

    threads.emplace_back([&nodes, timeout_timer]()
                         { sensor_node_DFSExecutor(nodes, "timer_4", "Visualizer", timeout_timer); });
    usleep(1000);

    threads.emplace_back([&nodes, timeout_timer]()
                         { sensor_node_DFSExecutor(nodes, "timer_5", "Lanelet2Map", timeout_timer); });
    usleep(1000);

    threads.emplace_back([&nodes, timeout_timer]()
                         { sensor_node_DFSExecutor(nodes, "timer_6", "EuclideanClusterSettings", timeout_timer); });
    usleep(1000);

    //  TRANSFORM NODES
    threads.emplace_back([&nodes, timeout_topic]()
                         { transform_node_DFSExecutor(nodes, "FrontLidarDriver", "PointsTransformerFront", timeout_topic); });
    usleep(1000);

    threads.emplace_back([&nodes, timeout_topic]()
                         { transform_node_DFSExecutor(nodes, "RearLidarDriver", "PointsTransformerRear", timeout_topic); });
    usleep(1000);

    threads.emplace_back([&nodes, timeout_topic]()
                         { transform_node_DFSExecutor(nodes, "PointCloudFusion", "VoxelGridDownsampler", timeout_topic); });
    usleep(1000);

    threads.emplace_back([&nodes, timeout_topic]()
                         { transform_node_DFSExecutor(nodes, "PointCloudMap", "PointCloudMapLoader", timeout_topic); });
    usleep(1000);

    threads.emplace_back([&nodes, timeout_topic]()
                         { transform_node_DFSExecutor(nodes, "PointCloudFusion", "RayGroundFilter", timeout_topic); });
    usleep(1000);

    threads.emplace_back([&nodes, timeout_topic]()
                         { transform_node_DFSExecutor(nodes, "EuclideanClusterDetector", "ObjectCollisionEstimator", timeout_topic); });
    usleep(1000);

    threads.emplace_back([&nodes, timeout_topic]()
                         { transform_node_DFSExecutor(nodes, "BehaviorPlanner", "MPCController", timeout_topic); });
    usleep(1000);

    threads.emplace_back([&nodes, timeout_topic]()
                         { transform_node_DFSExecutor(nodes, "Lanelet2MapLoader", "ParkingPlanner", timeout_topic); });
    usleep(1000);

    threads.emplace_back([&nodes, timeout_topic]()
                         { transform_node_DFSExecutor(nodes, "Lanelet2MapLoader", "LanePlanner", timeout_topic); });
    usleep(1000);

    //  FUSION NODES
    threads.emplace_back([&nodes, timeout_topic]()
                         { fusion_node_DFSExecutor(nodes, "PointsTransformerFront", "PointsTransformerRear", "PointCloudFusion", timeout_topic); });
    usleep(1000);

    threads.emplace_back([&nodes, timeout_topic]()
                         { fusion_node_DFSExecutor(nodes, "VoxelGridDownsampler", "PointCloudMapLoader", "NDTLocalizer", timeout_topic); });
    usleep(1000);

    threads.emplace_back([&nodes, timeout_topic]()
                         { fusion_node_DFSExecutor(nodes, "MPCController", "BehaviorPlanner", "VehicleInterface", timeout_topic); });
    usleep(1000);

    threads.emplace_back([&nodes, timeout_topic]()
                         { fusion_node_DFSExecutor(nodes, "Visualizer", "NDTLocalizer", "Lanelet2GlobalPlanner", timeout_topic); });
    usleep(1000);

    threads.emplace_back([&nodes, timeout_topic]()
                         { fusion_node_DFSExecutor(nodes, "Lanelet2Map", "Lanelet2GlobalPlanner", "Lanelet2MapLoader", timeout_topic); });
    usleep(1000);

    // CYCLIC NODES
    threads.emplace_back([&nodes, timeout_topic]()
                         { cyclic_node_DFSExecutor(nodes, "Cyclic_timer", "ObjectCollisionEstimator", "NDTLocalizer", "Lanelet2GlobalPlanner",
                                                   "Lanelet2MapLoader", "ParkingPlanner", "LanePlanner", "BehaviorPlanner", timeout_topic); });
    usleep(1000);

    // INTERSECTION NODES
    threads.emplace_back([&nodes, timeout_topic]()
                         { intersection_node_DFSExecutor(nodes, "EuclideanClusterDetector", "RayGroundFilter", "EuclideanClusterSettings",
                                                         "EuclideanClusterDetector", "EuclideanIntersection", timeout_topic); });
    usleep(1000);

    //  COMMAND NODES
    threads.emplace_back([&nodes, timeout_topic]()
                         { command_node_DFSExecutor(nodes, "VehicleInterface", "VehicleDBWSystem", timeout_topic); });
    usleep(1000);

    threads.emplace_back([&nodes, timeout_topic]()
                         { command_node_DFSExecutor(nodes, "EuclideanIntersection", "IntersectionOutput", timeout_topic); });
    usleep(1000);

    //  JOIN THREADS
    for (auto &thread : threads)
    {
        thread.join();
    }

    nodes.clear();
    rclcpp::shutdown();

    return 0;
}
