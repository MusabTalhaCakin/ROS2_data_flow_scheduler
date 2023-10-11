#pragma once

#include "reference_system/msg_types.hpp"
#include <rclcpp/rclcpp.hpp>

#include <vector>

using VectorSubcriptions =
    std::vector<rclcpp::Subscription<message_t>::SharedPtr>;

using VectorPublishers = std::vector<rclcpp::Publisher<message_t>::SharedPtr>;