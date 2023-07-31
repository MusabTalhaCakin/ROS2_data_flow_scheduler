/*******************************************************************************
 * Copyright (C) 2023 TTTech Auto AG. All rights reserved                      *
 * Operngasse 17-21, 1040 Vienna, Austria. office(at)tttech-auto.com           *
 ******************************************************************************/

#include <memory>
#include <vector>
#include <mutex>
#include <functional>
#include <string>
#include <chrono>
#include "data_flow_scheduler/data_flow_executor.h" // namespace DFS_Interface

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class TemplateNode : public rclcpp::Node
{
public:
  TemplateNode(const std::string &name_,
               const std::string &pub_name,
               const std::string &sub_name,
               std::vector<std::function<void()>> &functionVector,
               std::mutex &mutex,
               const int &runtime)
      : Node(name_), mutex_(mutex), functionVector_(functionVector), runtime_(runtime)
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        sub_name, 10, std::bind(&TemplateNode::topic_callback, this, _1));
    publisher_ = this->create_publisher<std_msgs::msg::String>(pub_name, 10);
    functionVector.push_back([]() {});
  }

  TemplateNode(const std::string &name_,
               const std::string &pub_name,
               std::vector<std::function<void()>> &functionVector,
               std::mutex &mutex)
      : Node(name_), mutex_(mutex), functionVector_(functionVector)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>(pub_name, 10);
    timer_ = this->create_wall_timer(
        0ms, std::bind(&TemplateNode::timer_callback, this));
    functionVector.push_back([]() {});
  }

  rclcpp::SubscriptionBase::SharedPtr return_subscription_()
  {
    return std::dynamic_pointer_cast<rclcpp::SubscriptionBase>(subscription_);
  }

  rclcpp::TimerBase::SharedPtr return_timer_()
  {
    return timer_;
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, World!";
    publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Published: '%s'", message.data.c_str());
  }

  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    auto message = std_msgs::msg::String();
    message.data = "Hello, World!";
    std::lock_guard<std::mutex> lock(mutex_);
    functionVector_[0] = [this, message]()
    {
      publisher_->publish(message);
      RCLCPP_INFO(this->get_logger(), "Published: '%s'", message.data.c_str());
    };
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
  std::mutex &mutex_;
  std::vector<std::function<void()>> &functionVector_;
  int runtime_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  std::vector<std::function<void()>> functionVector;
  std::mutex mutex;
  std::vector<std::string> arguments;

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

  switch (id)
  {
  case 0:
  {
    auto node = std::make_shared<TemplateNode>(arguments[0], arguments[1], functionVector, mutex);
    DFS_Interface::DFSExecutor executor(arguments[0],
                                        {{arguments[0], id, timeout, node->return_timer_(), 0, arguments[1]}});
    executor.spin(functionVector, mutex);
    break;
  }
  case 1:
  {
    auto node = std::make_shared<TemplateNode>(arguments[0], arguments[1], arguments[2], functionVector, mutex, timeout);
    DFS_Interface::DFSExecutor executor(arguments[0],
                                        {{arguments[2], id, timeout, node->return_subscription_(), 0, arguments[1]}});
    executor.spin(functionVector, mutex);
    break;
  }
  default:
    break;
  }

  rclcpp::shutdown();
  return 0;
}