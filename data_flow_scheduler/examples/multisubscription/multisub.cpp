/*******************************************************************************
 * Copyright (C) 2023 TTTech Auto AG. All rights reserved                      *
 * Operngasse 17-21, 1040 Vienna, Austria. office(at)tttech-auto.com           *
 ******************************************************************************/

#include "data_flow_scheduler/data_flow_executor.h" // namespace DFSched
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

using VectorSubcriptions = std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr>;

using VectorPublishers = std::vector<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr>;

class MultiSub : public rclcpp::Node
{
public:
  MultiSub(const std::string &name_, std::vector<std::string> &pub_names, const std::vector<std::string> &sub_names, std::mutex &mutex, const int &runtime)
      : Node(name_), mutex(mutex), runtime(runtime)
  {

    for (auto &sub_name : sub_names)
    {
      auto sub = create_subscription<std_msgs::msg::String>(sub_name, 10, std::bind(&MultiSub::topic_callback, this, _1));
      subscriptions.push_back(sub);
    }

    for (auto &pub_name : pub_names)
    {
      auto pub = create_publisher<std_msgs::msg::String>(pub_name, 10);
      publishers.push_back(pub);
    }
  }

  rclcpp::SubscriptionBase::SharedPtr get_subscription()
  {
    return std::dynamic_pointer_cast<rclcpp::SubscriptionBase>(subscriptions[0]);
  }

  void on_execution_callback()
  {
    std::cout << __FUNCTION__ << std::endl;
    std_msgs::msg::String msg;
    rclcpp::MessageInfo msg_info;

    uint64_t total = 0;
    // introduce some artificial latency to test DFSched::TimeSupervision::ThreadCPUTime supervision
    sleep(1);

    // Get all the topics
    for (auto sub_ptr : subscriptions)
    {
      std::cout << " Checking subscription: " << sub_ptr->get_topic_name() << "\n";
      if (sub_ptr->take(msg, msg_info))
      {
        std::shared_ptr<void> type_erased_msg = std::make_shared<std_msgs::msg::String>(msg);
        std::cout << " Value: " << msg.data.c_str() << "\n";
        total = total + atoll(msg.data.c_str());

        // If we don't call handle_message then
        // subscription callback is not called
        // sub_ptr->handle_message(type_erased_msg, msg_info);
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "   |->No message available");
      }
    }

    // Do the publishing
    for (auto pub_ptr : publishers)
    {
      auto message = std_msgs::msg::String();
      message.data = std::to_string(total);
      pub_ptr->publish(message);
      RCLCPP_DEBUG(this->get_logger(), "Published: '%s'", message.data.c_str());
    }
  }

private:
  // still need because ros2 needs a callback when a subscription is created
  // and can be used if handle_message() is executed
  void topic_callback(const std_msgs::msg::String::SharedPtr /*msg*/) const
  {
    std::cout << __FUNCTION__ << std::endl;
  }

  VectorSubcriptions subscriptions;
  VectorPublishers publishers;
  std::mutex &mutex;

  int runtime;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  std::mutex mutex;
  std::vector<std::string> arguments;

  std::vector<std::string> publish_topics{"topic3", "topic4"};
  std::vector<std::string> subscribe_topics{"topic1", "topic2"};

  int id = std::stoi(argv[1]);
  std::cout << "id : " << id << std::endl;
  int timeout = std::stoi(argv[2]);
  std::cout << "timeout : " << timeout << std::endl;
  for (int i = 3; i < argc; ++i)
  {
    arguments.push_back(argv[i]);
  }
  for (unsigned int i = 0; i < arguments.size(); i++)
  {
    std::cout << arguments[i] << std::endl;
  }

  auto node = std::make_shared<MultiSub>("Node3", publish_topics, subscribe_topics, mutex, timeout);

  DFSched::CallbackInfoVector vinfo(1);
  vinfo[0].subs = subscribe_topics;
  vinfo[0].pubs = publish_topics;
  vinfo[0].supervision_kind = DFSched::TimeSupervision::ThreadCPUTime;
  vinfo[0].runtime = timeout; // in microseconds

  //  here we define the callback that is going to be called
  //  when precedesors are done
  vinfo[0].callback_ptr = [&node]()
  { node->on_execution_callback(); };

  vinfo[0].id = 0;

  DFSched::DFSExecutor executor(std::string("Node3"), vinfo);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
