// Copyright 2021 Apex.AI, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef AUTOWARE_REFERENCE_SYSTEM__AUTOWARE_SYSTEM_BUILDER_HPP_
#define AUTOWARE_REFERENCE_SYSTEM__AUTOWARE_SYSTEM_BUILDER_HPP_
#include <chrono>
#include <memory>
#include <vector>
#include <string>

#include "reference_system/nodes/settings.hpp"
#include "reference_system/sample_management.hpp"

using namespace std::chrono_literals;  // NOLINT

template<typename SystemType, typename TimingConfig>
auto create_autoware_nodes()
->std::vector<std::shared_ptr<typename SystemType::NodeBaseType>>
{
  std::vector<std::shared_ptr<typename SystemType::NodeBaseType>> nodes;

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

// ignore the warning about designated initializers - they make the code much
// more readable
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"

  // setup communication graph
  // sensor nodes
  nodes.emplace_back(
    std::make_shared<typename SystemType::Sensor>(
      nodes::SensorSettings{.node_name = "FrontLidarDriver",
        .topic_name = "FrontLidarDriver",
        .cycle_time = TimingConfig::FRONT_LIDAR_DRIVER}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Sensor>(
      nodes::SensorSettings{.node_name = "RearLidarDriver",
        .topic_name = "RearLidarDriver",
        .cycle_time = TimingConfig::REAR_LIDAR_DRIVER}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Sensor>(
      nodes::SensorSettings{.node_name = "PointCloudMap",
        .topic_name = "PointCloudMap",
        .cycle_time = TimingConfig::POINT_CLOUD_MAP}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Sensor>(
      nodes::SensorSettings{.node_name = "Visualizer",
        .topic_name = "Visualizer",
        .cycle_time = TimingConfig::VISUALIZER}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Sensor>(
      nodes::SensorSettings{.node_name = "Lanelet2Map",
        .topic_name = "Lanelet2Map",
        .cycle_time = TimingConfig::LANELET2MAP}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Sensor>(
      nodes::SensorSettings{
    .node_name = "EuclideanClusterSettings",
    .topic_name = "EuclideanClusterSettings",
    .cycle_time = TimingConfig::EUCLIDEAN_CLUSTER_SETTINGS}));

  // transform nodes
  nodes.emplace_back(
    std::make_shared<typename SystemType::Transform>(
      nodes::TransformSettings{
    .node_name = "PointsTransformerFront",
    .input_topic = "FrontLidarDriver",
    .output_topic = "PointsTransformerFront",
    .number_crunch_limit = TimingConfig::POINTS_TRANSFORMER_FRONT}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Transform>(
      nodes::TransformSettings{
    .node_name = "PointsTransformerRear",
    .input_topic = "RearLidarDriver",
    .output_topic = "PointsTransformerRear",
    .number_crunch_limit = TimingConfig::POINTS_TRANSFORMER_REAR}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Transform>(
      nodes::TransformSettings{
    .node_name = "VoxelGridDownsampler",
    .input_topic = "PointCloudFusion",
    .output_topic = "VoxelGridDownsampler",
    .number_crunch_limit = TimingConfig::VOXEL_GRID_DOWNSAMPLER}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Transform>(
      nodes::TransformSettings{
    .node_name = "PointCloudMapLoader",
    .input_topic = "PointCloudMap",
    .output_topic = "PointCloudMapLoader",
    .number_crunch_limit = TimingConfig::POINT_CLOUD_MAP_LOADER}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Transform>(
      nodes::TransformSettings{
    .node_name = "RayGroundFilter",
    .input_topic = "PointCloudFusion",
    .output_topic = "RayGroundFilter",
    .number_crunch_limit = TimingConfig::RAY_GROUND_FILTER}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Transform>(
      nodes::TransformSettings{
    .node_name = "ObjectCollisionEstimator",
    .input_topic = "EuclideanClusterDetector",
    .output_topic = "ObjectCollisionEstimator",
    .number_crunch_limit = TimingConfig::OBJECT_COLLISION_ESTIMATOR}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Transform>(
      nodes::TransformSettings{
    .node_name = "MPCController",
    .input_topic = "BehaviorPlanner",
    .output_topic = "MPCController",
    .number_crunch_limit = TimingConfig::MPC_CONTROLLER}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Transform>(
      nodes::TransformSettings{
    .node_name = "ParkingPlanner",
    .input_topic = "Lanelet2MapLoader",
    .output_topic = "ParkingPlanner",
    .number_crunch_limit = TimingConfig::PARKING_PLANNER}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Transform>(
      nodes::TransformSettings{
    .node_name = "LanePlanner",
    .input_topic = "Lanelet2MapLoader",
    .output_topic = "LanePlanner",
    .number_crunch_limit = TimingConfig::LANE_PLANNER}));

  // fusion nodes
  nodes.emplace_back(
    std::make_shared<typename SystemType::Fusion>(
      nodes::FusionSettings{
    .node_name = "PointCloudFusion",
    .input_0 = "PointsTransformerFront",
    .input_1 = "PointsTransformerRear",
    .output_topic = "PointCloudFusion",
    .number_crunch_limit = TimingConfig::POINT_CLOUD_FUSION}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Fusion>(
      nodes::FusionSettings{
    .node_name = "NDTLocalizer",
    .input_0 = "VoxelGridDownsampler",
    .input_1 = "PointCloudMapLoader",
    .output_topic = "NDTLocalizer",
    .number_crunch_limit = TimingConfig::NDT_LOCALIZER}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Fusion>(
      nodes::FusionSettings{
    .node_name = "VehicleInterface",
    .input_0 = "MPCController",
    .input_1 = "BehaviorPlanner",
    .output_topic = "VehicleInterface",
    .number_crunch_limit = TimingConfig::VEHICLE_INTERFACE}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Fusion>(
      nodes::FusionSettings{
    .node_name = "Lanelet2GlobalPlanner",
    .input_0 = "Visualizer",
    .input_1 = "NDTLocalizer",
    .output_topic = "Lanelet2GlobalPlanner",
    .number_crunch_limit = TimingConfig::LANELET_2_GLOBAL_PLANNER}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Fusion>(
      nodes::FusionSettings{
    .node_name = "Lanelet2MapLoader",
    .input_0 = "Lanelet2Map",
    .input_1 = "Lanelet2GlobalPlanner",
    .output_topic = "Lanelet2MapLoader",
    .number_crunch_limit = TimingConfig::LANELET_2_MAP_LOADER}));

  // cyclic node
  nodes.emplace_back(
    std::make_shared<typename SystemType::Cyclic>(
      nodes::CyclicSettings{
    .node_name = "BehaviorPlanner",
    .inputs = {"ObjectCollisionEstimator", "NDTLocalizer",
      "Lanelet2GlobalPlanner", "Lanelet2MapLoader",
      "ParkingPlanner", "LanePlanner"},
    .output_topic = "BehaviorPlanner",
    .number_crunch_limit = TimingConfig::BEHAVIOR_PLANNER,
    .cycle_time = TimingConfig::BEHAVIOR_PLANNER_CYCLE}));

  // intersection node
  nodes.emplace_back(
    std::make_shared<typename SystemType::Intersection>(
      nodes::IntersectionSettings{
    .node_name = "EuclideanClusterDetector",
    .connections = {
      {.input_topic = "RayGroundFilter",
        .output_topic = "EuclideanClusterDetector",
        .number_crunch_limit = TimingConfig::EUCLIDEAN_CLUSTER_DETECTOR},
      {.input_topic = "EuclideanClusterSettings",
        .output_topic = "EuclideanIntersection",
        .number_crunch_limit = TimingConfig::EUCLIDEAN_INTERSECTION}}}));

  // command node
  nodes.emplace_back(
    std::make_shared<typename SystemType::Command>(
      nodes::CommandSettings{
    .node_name = "VehicleDBWSystem", .input_topic = "VehicleInterface"}));

  nodes.emplace_back(
    std::make_shared<typename SystemType::Command>(
      nodes::CommandSettings{.node_name = "IntersectionOutput",
        .input_topic = "EuclideanIntersection"}));
#pragma GCC diagnostic pop

  return nodes;
}

template<typename NodeType>
std::shared_ptr<NodeType> get_node(
  const std::string & name,
  const std::vector<std::shared_ptr<NodeType>> & v)
{
  for (const auto & n : v) {
    if (n->get_name() == std::string(name)) {
      return n;
    }
  }

  return std::shared_ptr<NodeType>();
}

#endif  // AUTOWARE_REFERENCE_SYSTEM__AUTOWARE_SYSTEM_BUILDER_HPP_
