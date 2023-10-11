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
#ifndef REFERENCE_SYSTEM__NODES__RCLCPP__FUSION_HPP_
#define REFERENCE_SYSTEM__NODES__RCLCPP__FUSION_HPP_
#include <chrono>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "reference_system/msg_types.hpp"
#include "reference_system/nodes/rclcpp/common.hpp"
#include "reference_system/nodes/settings.hpp"
#include "reference_system/number_cruncher.hpp"
#include "reference_system/sample_management.hpp"

namespace nodes
{
  namespace rclcpp_system
  {

    class Fusion : public rclcpp::Node
    {
    public:
      explicit Fusion(const FusionSettings &settings)
          : Node(settings.node_name),
            number_crunch_limit_(settings.number_crunch_limit)
      {
        auto sub = create_subscription<message_t>(
            settings.input_0, 1,
            std::bind(&Fusion::process_callback, this, std::placeholders::_1));
        auto sub2 = create_subscription<message_t>(
            settings.input_1, 1,
            std::bind(&Fusion::process_callback, this, std::placeholders::_1));

        subscriptions.push_back(sub);
        subscriptions.push_back(sub2);
        RCLCPP_INFO(get_logger(), "Subscription1 '%s' Subs2 '%s' Publishing '%s'",
                    settings.input_0.c_str(), settings.input_1.c_str(),
                    settings.output_topic.c_str());

        auto pub = create_publisher<message_t>(settings.output_topic, 1);
        publishers.push_back(pub);
      }

      rclcpp::SubscriptionBase::SharedPtr return_subscription_1() const
      {
        return subscriptions[0];
      }
      rclcpp::SubscriptionBase::SharedPtr return_subscription_2() const
      {
        return subscriptions[1];
      }

      void another_callback()
      {
        // std::cout << __FUNCTION__ << std::endl;
        uint64_t timestamp = now_as_int();
        message_t msg;
        rclcpp::MessageInfo msg_info;
        uint32_t missed_samples = 0;
        auto number_cruncher_result = number_cruncher(number_crunch_limit_);
        auto output_message = publishers[0]->borrow_loaned_message();
        output_message.get().size = 0;
        // Get all the topics
        for (auto sub_ptr : subscriptions)
        {
          // std::cout << " Checking subscription: " << sub_ptr->get_topic_name() << "\n";
          if (sub_ptr->take(msg, msg_info))
          {
            message_t::SharedPtr type_erased_msg = std::make_shared<message_t>(msg);
            missed_samples += get_missed_samples_and_update_seq_nr(type_erased_msg, input_sequence_number_);
            merge_history_into_sample(output_message.get(), type_erased_msg);
            // sub_ptr->handle_message(type_erased_msg2, msg_info);
          }
          else
          {
            RCLCPP_WARN(this->get_logger(), "   |->No message available %s", sub_ptr->get_topic_name());
          }
        }

        set_sample(this->get_name(), sequence_number_++, missed_samples, timestamp,
                   output_message.get());

        // use result so that it is not optimizied away by some clever compiler
        output_message.get().data[0] = number_cruncher_result;

        publishers[0]->publish(std::move(output_message));
        // RCLCPP_WARN(this->get_logger(), "******* Published topic %s",
        // publishers[0]->get_topic_name());
        // std::cout << "~" << __FUNCTION__ <<   std::endl;
      }

    private:
      void process_callback(const message_t::SharedPtr /*input_message*/)
      {
        std::cout << __FUNCTION__ << std::endl;
      }

      void input_callback(const uint64_t /*input_number*/,
                          const message_t::SharedPtr /*input_message*/)
      {
      }

    private:
      uint64_t number_crunch_limit_;
      uint32_t sequence_number_ = 0;
      std::mutex mutex_;
      uint32_t input_sequence_number_ = 0;
      VectorSubcriptions subscriptions;
      VectorPublishers publishers;
    };
  } // namespace rclcpp_system
} // namespace nodes
#endif // REFERENCE_SYSTEM__NODES__RCLCPP__FUSION_HPP_