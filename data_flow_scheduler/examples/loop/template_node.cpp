/*******************************************************************************
 * Copyright (C) 2023 TTTech Auto AG. All rights reserved                      *
 * Operngasse 17-21, 1040 Vienna, Austria. office(at)tttech-auto.com           *
 ******************************************************************************/

#include "data_flow_scheduler/data_flow_executor.h" // namespace DFSched
#include <algorithm>
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

using VectorSubcriptions = std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr>;

using VectorPublishers = std::vector<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr>;

class SimpleNode : public rclcpp::Node
{
public:
  SimpleNode(const std::string &name_, std::vector<std::string> &pub_names, const std::vector<std::string> &sub_names, std::mutex &mutex, const int &runtime)
      : Node(name_), mutex(mutex), runtime(runtime)
  {

    for (auto &sub_name : sub_names)
    {
      auto sub = create_subscription<std_msgs::msg::String>(sub_name, 10, std::bind(&SimpleNode::topic_callback, this, _1));
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

    uint64_t i = 0;
    // Get all the topics
    for (auto sub_ptr : subscriptions)
    {
      std::cout << " Checking subscription: " << sub_ptr->get_topic_name() << "\n";
      if (sub_ptr->take(msg, msg_info))
      {
        std::shared_ptr<void> type_erased_msg = std::make_shared<std_msgs::msg::String>(msg);
        std::cout << " Value: " << msg.data.c_str() << "\n";
        i += atoll(msg.data.c_str()) * 2;

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
      message.data = std::to_string(i);
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

    /*do
    {
      std::cout << "Press [ENTER] to send topic1\n";
    } while (std::cin.get() != '\n');*/

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

    /*do
    {
      std::cout << "Press [ENTER] to send topic2\n";
    } while (std::cin.get() != '\n');*/

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
    auto node = std::make_shared<TemplateNode>(arguments[0], arguments[2], arguments[1], mutex, timeout);

    // DFSched::TopicInfoVector is used for nodes that just subcribes to 1 topic and publishes 1 topic
    DFSched::TopicInfoVector vtopic(1);
    vtopic[0].name = arguments[1];
    vtopic[0].type = id;
    vtopic[0].runtime = timeout;
    vtopic[0].sub_ptr = node->get_subscription();
    vtopic[0].id = 0;
    vtopic[0].pub = arguments[2];
    vtopic[0].supervision_kind = DFSched::TimeSupervision::Time;

    DFSched::DFSExecutor executor(arguments[0], vtopic);
    executor.spin();
    break;
  }
  case 2:
  {
    std::vector<std::string> publish_topics{arguments.back()};
    std::vector<std::string> subscribe_topics;

    std::copy(arguments.begin() + 1, arguments.end() - 1, std::back_inserter(subscribe_topics));

    auto node = std::make_shared<SimpleNode>(arguments.at(0), publish_topics, subscribe_topics, mutex, timeout);

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

    DFSched::DFSExecutor executor(arguments.at(0), vinfo);
    executor.spin();
  }
  default:
    break;
  }

  rclcpp::shutdown();
  return 0;
}