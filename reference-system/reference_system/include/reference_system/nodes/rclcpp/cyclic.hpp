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
#ifndef REFERENCE_SYSTEM__NODES__RCLCPP__CYCLIC_HPP_
#define REFERENCE_SYSTEM__NODES__RCLCPP__CYCLIC_HPP_
#include <chrono>
#include <rcutils/logging.h>
#include <string>
#include <utility>
#include <vector>

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

    class Cyclic : public rclcpp::Node
    {
    public:
      explicit Cyclic(const CyclicSettings &settings)
          : Node(settings.node_name),
            number_crunch_limit_(settings.number_crunch_limit)
      {

        uint64_t input_number = 0U;
        for (const auto &input_topic : settings.inputs)
        {

          auto sub = create_subscription<message_t>(
              input_topic, 1, [](const message_t::SharedPtr /*input_message*/) {});

          subscriptions.push_back(sub);
          ++input_number;
        }

        auto pub = create_publisher<message_t>(settings.output_topic, 1);
        publishers.push_back(pub);

        timer_ = this->create_wall_timer(settings.cycle_time,
                                         [this]
                                         { timer_callback(); });
      }

      rclcpp::SubscriptionBase::SharedPtr return_subscription_1()
      {
        return subscriptions_[0].subscription;
      }
      rclcpp::SubscriptionBase::SharedPtr return_subscription_2()
      {
        return subscriptions_[1].subscription;
      }
      rclcpp::SubscriptionBase::SharedPtr return_subscription_3()
      {
        return subscriptions_[2].subscription;
      }
      rclcpp::SubscriptionBase::SharedPtr return_subscription_4()
      {
        return subscriptions_[3].subscription;
      }
      rclcpp::SubscriptionBase::SharedPtr return_subscription_5()
      {
        return subscriptions_[4].subscription;
      }
      rclcpp::SubscriptionBase::SharedPtr return_subscription_6()
      {
        return subscriptions_[5].subscription;
      }
      rclcpp::TimerBase::SharedPtr return_timer_() { return timer_; }

    private:
      void input_callback(const uint64_t input_number,
                          const message_t::SharedPtr input_message)
      {
        subscriptions_[input_number].cache = input_message;
      }

    public:
      void timer_callback()
      {
        // std::cout << __FUNCTION__ << std::endl;
        uint64_t timestamp = now_as_int();
        message_t msg;
        rclcpp::MessageInfo msg_info;
        auto &publisher_ = publishers[0];
        auto number_cruncher_result = number_cruncher(number_crunch_limit_);

        auto output_message = publisher_->borrow_loaned_message();
        output_message.get().size = 0;

        uint32_t missed_samples = 0;

        // Get all the topics
        for (auto sub_ptr : subscriptions)
        {
          // std::cout << " Checking subscription: " << sub_ptr->get_topic_name() <<
          // "\n";
          if (sub_ptr->take(msg, msg_info))
          {
            message_t::SharedPtr type_erased_msg = std::make_shared<message_t>(msg);
            missed_samples += get_missed_samples_and_update_seq_nr(type_erased_msg, input_sequence_number_);
            merge_history_into_sample(output_message.get(), type_erased_msg);

            // sub_ptr->handle_message(type_erased_msg2, msg_info);
          }
          else
          {
            RCLCPP_WARN(this->get_logger(), "   |->No message available %s",
                        sub_ptr->get_topic_name());
          }
        }

        set_sample(this->get_name(), sequence_number++, missed_samples, timestamp,
                   output_message.get());

        output_message.get().data[0] = number_cruncher_result;
        publisher_->publish(std::move(output_message));
        // RCLCPP_DEBUG(this->get_logger(), "******* Published topic %s",
        // publisher_->get_topic_name());
        //  std::cout << "~" << __FUNCTION__ << std::endl;
      }

    private:
      rclcpp::TimerBase::SharedPtr timer_;

      struct subscription_t
      {
        rclcpp::Subscription<message_t>::SharedPtr subscription;
        uint32_t sequence_number = 0;
        message_t::SharedPtr cache;
      };

      std::vector<subscription_t> subscriptions_;
      uint64_t number_crunch_limit_;
      uint32_t sequence_number = 0;
      uint32_t input_sequence_number_ = 0;

      VectorSubcriptions subscriptions;
      VectorPublishers publishers;
    };
  } // namespace rclcpp_system
} // namespace nodes
#endif // REFERENCE_SYSTEM__NODES__RCLCPP__CYCLIC_HPP_
