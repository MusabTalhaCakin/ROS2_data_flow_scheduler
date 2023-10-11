/*******************************************************************************
 * Copyright (C) 2023 TTTech Auto AG. All rights reserved                      *
 * Operngasse 17-21, 1040 Vienna, Austria. office(at)tttech-auto.com           *
 ******************************************************************************/

#include "data_flow_scheduler/data_flow_executor.h" // namespace DFSched
#include <chrono>
#include <cstdlib>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class TemplateNode : public rclcpp::Node
{
public:
  TemplateNode(const std::string &name_, const std::string &pub_name, const std::string &sub_name,
               std::mutex &mutex, const int &runtime)
      : Node(name_), mutex(mutex), runtime(runtime)
  {
    subscription = this->create_subscription<std_msgs::msg::String>(sub_name, 10, std::bind(&TemplateNode::topic_callback, this, _1));
    publisher = this->create_publisher<std_msgs::msg::String>(pub_name, 10);
  }

  TemplateNode(const std::string &name_, const std::string &pub_name, std::mutex &mutex)
      : Node(name_), mutex(mutex)
  {
    publisher = this->create_publisher<std_msgs::msg::String>(pub_name, 10);
    timer = this->create_wall_timer(0ms, std::bind(&TemplateNode::timer_callback, this));
  }

  rclcpp::SubscriptionBase::SharedPtr get_subscription()
  {
    return std::dynamic_pointer_cast<rclcpp::SubscriptionBase>(subscription);
  }

  rclcpp::TimerBase::SharedPtr get_timer() { return timer; }

private:
  void timer_callback()
  {
    static uint64_t i = 0;
    auto message = std_msgs::msg::String();

    do
    {
      std::cout << "Press [ENTER] to send topic1\n";
    } while (std::cin.get() != '\n');

    message.data = std::to_string(i);
    publisher->publish(message);
    RCLCPP_DEBUG(this->get_logger(), "Published by timer: '%s'", message.data.c_str());
    ++i;
  }

  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_DEBUG(this->get_logger(), "I heard: '%s'", msg->data.c_str());

    uint64_t i = atoll(msg->data.c_str());
    i = i * 2;

    do
    {
      std::cout << "Press [ENTER] to send topic2\n";
    } while (std::cin.get() != '\n');

    auto message = std_msgs::msg::String();
    message.data = std::to_string(i);
    std::lock_guard<std::mutex> lock(mutex);

    publisher->publish(message);
    RCLCPP_DEBUG(this->get_logger(), "Published '%s'", message.data.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
  rclcpp::TimerBase::SharedPtr timer;

  std::mutex &mutex;

  int runtime;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

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
    auto node = std::make_shared<TemplateNode>(arguments[0], arguments[1], mutex);

    // DFSched::TimerInfoVector is used by Nodes triggered by a timer
    DFSched::TimerInfoVector vtime(1);
    vtime[0].id = id;
    vtime[0].pub = arguments[1];
    vtime[0].runtime = timeout;
    vtime[0].time_ptr = node->get_timer();
    vtime[0].type = id;
    vtime[0].supervision_kind = DFSched::TimeSupervision::Time;

    DFSched::DFSExecutor executor(arguments[0], vtime);
    executor.spin();
    break;
  }
  case 1:
  {
    auto node = std::make_shared<TemplateNode>(arguments[0], arguments[1], arguments[2], mutex, timeout);

    // DFSched::TopicInfoVector is used for nodes that just subcribes to 1 topic and publishes 1 topic
    DFSched::TopicInfoVector vtopic(1);
    vtopic[0].name = arguments[2];
    vtopic[0].type = id;
    vtopic[0].runtime = timeout;
    vtopic[0].sub_ptr = node->get_subscription();
    vtopic[0].id = 0;
    vtopic[0].pub = arguments[1];
    vtopic[0].supervision_kind = DFSched::TimeSupervision::Time;

    DFSched::DFSExecutor executor(arguments[0], vtopic);
    executor.spin();
    break;
  }
  default:
    break;
  }

  rclcpp::shutdown();
  return 0;
}