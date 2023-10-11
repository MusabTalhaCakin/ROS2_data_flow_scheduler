/*******************************************************************************
 * Copyright (C) 2023 TTTech Auto AG. All rights reserved                      *
 * Operngasse 17-21, 1040 Vienna, Austria. office(at)tttech-auto.com           *
 ******************************************************************************/
#include <chrono>
#include <iostream>
#include <string>

#include "data_flow_scheduler/data_flow_executor.h" // namespace DFSched
#include "data_flow_scheduler/data_flow_types.h"

static auto has_thread_timeout(pthread_t thread_id, int runtime) -> bool;
static auto get_cpu_time_spent_us(pthread_t thread_id) -> std::int64_t;

using namespace DFSched;

DFSExecutor::DFSExecutor(const std::string node_name_, TimerInfoVector Timers_)
{
  node_name = node_name_;
  timers = Timers_;
  init();
}

DFSExecutor::DFSExecutor(const std::string node_name_, TopicInfoVector Callbacks_)
{
  node_name = node_name_;
  topics_ = Callbacks_;
  init();
}

DFSExecutor::DFSExecutor(const std::string node_name_, TimerInfoVector Timers_, TopicInfoVector Callbacks_)
{
  node_name = node_name_;
  timers = Timers_;
  topics_ = Callbacks_;
  init();
}

DFSExecutor::DFSExecutor(const std::string &node_name, CallbackInfoVector callbacks)
    : node_name(node_name), callbacks(callbacks)
{
  init();
}

// Initialize the DFSExecutor object
void DFSExecutor::init()
{
  std::cout << "DFSExecutor Info:\n--\n";
  for (const auto &timer : timers)
  {
    // Store timer information
    node_info.callback_topic_name.push_back(timer.name);
    node_info.runtime.push_back(timer.runtime);
    node_info.callback_id.push_back(timer.id);
    node_info.callback_type.push_back(timer.type);
    node_info.pub_topic_name.push_back(timer.pub);
    node_info.supervision_kind = timer.supervision_kind;

    // Print timer information
    std::cout << "TIMER: \n";
    std::cout << " Name: " << timer.name << std::endl;
    std::cout << " Runtime: " << timer.runtime << std::endl;
    std::cout << " ID: " << timer.id << std::endl;
    std::cout << " Type: " << timer.type << std::endl;
    std::cout << "PUB: " << timer.pub << std::endl;
    std::cout << "Supervision: " << static_cast<uint32_t>(node_info.supervision_kind) << "\n";
  }

  for (const auto &callback : topics_)
  {
    // Store callback information
    node_info.callback_topic_name.push_back(callback.name);
    node_info.runtime.push_back(callback.runtime);
    node_info.callback_id.push_back(callback.id);
    node_info.callback_type.push_back(callback.type);
    node_info.pub_topic_name.push_back(callback.pub);
    node_info.supervision_kind = callback.supervision_kind;
    // Print callback information
    std::cout << "TOPIC: \n";
    std::cout << " Name: " << callback.name << std::endl;
    std::cout << " Runtime: " << callback.runtime << std::endl;
    std::cout << " ID: " << callback.id << std::endl;
    std::cout << " Type: " << callback.type << std::endl;
    std::cout << "PUB: " << callback.pub << std::endl;
    std::cout << "Supervision: " << static_cast<uint32_t>(node_info.supervision_kind) << "\n";
  }

  for (const auto &callback : callbacks)
  {
    // Store callback information
    node_info.callback_topic_name = callback.subs;
    node_info.runtime.push_back(callback.runtime);
    node_info.callback_id.push_back(callback.id);
    node_info.callback_type.push_back(callback.type);
    node_info.pub_topic_name = callback.pubs;
    node_info.supervision_kind = callback.supervision_kind;
    // Print callback information
    std::cout << "TOPIC: \n";
    std::cout << " Names: ";
    for (auto &s : callback.subs)
    {
      std::cout << s << ", ";
    }

    std::cout << "\n";

    std::cout << " Runtime: " << callback.runtime << std::endl;
    std::cout << " ID: " << callback.id << std::endl;
    std::cout << " Type: " << callback.type << std::endl;
    std::cout << "PUB: " << std::endl;
    for (auto &s : callback.pubs)
    {
      std::cout << s << ", ";
    }
    std::cout << "\n";
  }

  std::cout << "\n--\n";

  RCLCPP_INFO(rclcpp::get_logger(node_name), "DFSExecutor Created.");

  // Initialize the DFS executor with callbacks and timers
  cb_handler.init(node_name, topics_, timers, callbacks);

  // Connect to the server
  if (!client.connect(node_name))
  {
    RCLCPP_ERROR(rclcpp::get_logger(node_name + "|DFSExecutor"),
                 "Connecting to Server failed");
  }

  if (!send_callback_infos())
  {
    RCLCPP_ERROR(rclcpp::get_logger(node_name + "|DFSExecutor"),
                 "Sending Infos message to Server failed");
  }
}

// Spin function that runs in a separate thread and executes read_msgs based on
// incoming Execute_Info
void DFSExecutor::spin()
{

  std::vector<std::thread *> child_thread(node_info.callback_topic_name.size());

  while (true)
  {
    ExecutionParams params;
    // RCLCPP_WARN(rclcpp::get_logger(node_name + "|DFSExecutor"), "Waiting for
    // reading data");

    int read_ = client.read_raw_data(&params, sizeof(ExecutionParams));

    /*RCLCPP_WARN(rclcpp::get_logger(node_name + "|DFSExecutor"),
                "Done reading data");

    std::cout << "DATA: \n";
    std::cout << " |-> mtx_id: " << params.mtx_id << "\n";
    std::cout << " |-> callb: " << params.callb << "\n";
    std::cout << " |-> pr: " << params.pr << "\n";
    std::cout << " |-> runtime: " << params.runtime << "\n";
    std::cout << " |-> suc: " << params.suc << "\n";
    std::cout << " |-> timeout: " << params.timeout << "\n";
    std::cout << " |-> type: " << params.type << "\n";
    std::cout << " |-> supervision: " << static_cast<uint32_t>(params.supervision_kind) << "\n";*/

    int j = params.mtx_id;

    if (read_ < 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger(node_name + "|DFSExecutor"),
                   "Could not read from socket.");
      break;
    }
    else
    {
      if (child_thread[j] != nullptr)
      {
        if (child_thread[j]->joinable())
        {
          // Detach the previously joined child thread
          // std::cout << "Detaching thread " << j << "\n";
          // RCLCPP_INFO(rclcpp::get_logger(node_name + "|DFSExecutor"),
          // "Detaching thread %d", j);
          auto pt = child_thread[j];
          detached_threads.push_back(pt);
          pt->detach();

          child_thread[j] = nullptr;
        }
      }
      // Create a new child thread to execute the read_msgs function
      child_thread[j] =
          new std::thread(&DFSExecutor::read_msgs, this, params);

#if defined(SET_THREAD_PRIORITY) && (SET_THREAD_PRIORITY == true)
      if (child_thread[j] != nullptr)
      {
        setThreadPriority(*child_thread[j], DEFAULT_MONITOR_PRIO,
                          DEFAULT_MONITOR_POLICY, node_name);
      }
#endif
    }
  }
}

// Serialize the buffer containing callback information
std::string DFSExecutor::serialize_buffer() const
{
  std::string result;

  for (const auto &topic_name : node_info.callback_topic_name)
  {
    result += topic_name + ",";
  }
  result += ";";
  for (const auto &runtime : node_info.runtime)
  {
    result += std::to_string(runtime) + ",";
  }
  result += ";";
  for (const auto &topic_name : node_info.pub_topic_name)
  {
    result += topic_name + ",";
  }
  result += ";";
  for (const auto &id : node_info.callback_id)
  {
    result += std::to_string(id) + ",";
  }
  result += ";";
  for (const auto &type : node_info.callback_type)
  {
    result += std::to_string(type) + ",";
  }
  result += ";";
  result += std::to_string(static_cast<std::uint32_t>(node_info.supervision_kind)) + ",";
  return result;
}

// Send callback information to the server
bool DFSExecutor::send_callback_infos()
{
  std::string n_buffer = serialize_buffer();
  return client.send_data(n_buffer);
}

// Function executed by child threads to read messages and execute callbacks
void DFSExecutor::read_msgs(ExecutionParams params)
{

  std::unique_lock<std::mutex> lock(cb_handler.timeout_condition[params.mtx_id]->mtx_);
  cb_handler.timeout_condition[params.mtx_id]->finished_.store(false);

  std::thread thr_cbk(&DFSched::CallbackHandler::run_callback, &cb_handler, params.mtx_id, params.callb, params.type);

#if defined(SET_THREAD_PRIORITY) && (SET_THREAD_PRIORITY == true)
  setThreadPriority(thr_cbk, DEFAULT_TASK_PRIO, DEFAULT_TASK_POLICY, node_name);
#endif

  bool finish = false;
  auto time_point = std::chrono::high_resolution_clock::now();
  bool has_timeout = false;
  params.timeout = false;

  auto t_start = std::chrono::system_clock::now();
  do
  {

    // calculate how much time elapsed
    int elapsed = std::chrono::duration<float, std::micro>(std::chrono::system_clock::now() - time_point).count();

    if (true == cb_handler.timeout_condition[params.mtx_id]->finished_.load())
    {
      finish = true;
    }
    else if (elapsed >= params.runtime)
    {

      switch (params.supervision_kind)
      {
      case TimeSupervision::Time:
        has_timeout = true;
        break;
      case TimeSupervision::None:
        has_timeout = false;
        break;
      case TimeSupervision::ThreadCPUTime:
        has_timeout = has_thread_timeout(thr_cbk.native_handle(), params.runtime);
        break;
      default:
        has_timeout = true;
        break;
      }

#if defined(ENABLE_TIMEOUT) && (ENABLE_TIMEOUT == true)
      finish = true;
#else
      time_point = std::chrono::high_resolution_clock::now();
#endif
    }
    else
    {
      /* Calculate deadline time */
      auto wait_time = std::chrono::microseconds(params.runtime - elapsed);
      switch (params.supervision_kind)
      {
      default:
      case TimeSupervision::Time:
        wait_time = std::chrono::microseconds(params.runtime - elapsed);
        break;
      case TimeSupervision::None:
        // wait forever until finish
        wait_time = std::chrono::microseconds(params.runtime);
        break;
      case TimeSupervision::ThreadCPUTime:
        auto spent_us = get_cpu_time_spent_us(thr_cbk.native_handle());
        if (spent_us > -1)
        {
          wait_time = std::chrono::microseconds(params.runtime - spent_us);
        }
        break;
      }

      auto timep = std::chrono::system_clock::now() + wait_time;
      /* Wait until next activation */
      // RCLCPP_WARN(rclcpp::get_logger(node_name + "|DFSExecutor"), "Waiting for activation");
      auto is_timeout = (std::cv_status::timeout == cb_handler.timeout_condition[params.mtx_id]->cvar_.wait_until(lock, timep));

      if (params.supervision_kind != TimeSupervision::Time)
      {
        // check again time spent by cpu thread
        auto spent_us = get_cpu_time_spent_us(thr_cbk.native_handle());
        is_timeout = (spent_us > params.runtime || spent_us == -1) && params.supervision_kind == TimeSupervision::ThreadCPUTime;
      }

      // RCLCPP_WARN(rclcpp::get_logger(node_name + "|DFSExecutor"),
      // "Activation!");
      finish = cb_handler.timeout_condition[params.mtx_id]->finished_.load();

      has_timeout = (is_timeout && (!finish)) || has_timeout;
#if defined(ENABLE_TIMEOUT) && (ENABLE_TIMEOUT == true)
      finish = (finish || is_timeout);
#endif
    }
  } while (false == finish);

  if (true == has_timeout)
  {
    auto t_elapsed = std::chrono::duration<double, std::micro>(
                         std::chrono::system_clock::now() - t_start)
                         .count();
    RCLCPP_WARN(rclcpp::get_logger(node_name),
                "Timeout occurred. Function did not finish in time!. elapsed: "
                "%d, runtime: %d",
                int(t_elapsed), params.runtime);
#if defined(ENABLE_TIMEOUT) && (ENABLE_TIMEOUT == true)
    params.timeout = true;
#endif
  }

  /* Report status */
  params.suc = cb_handler.timeout_condition[params.mtx_id]->suc_;
  // RCLCPP_WARN(rclcpp::get_logger(node_name + "|DFSExecutor"), "Sending
  // data!");
  client.send_raw_data(&params, sizeof(ExecutionParams));

  if (thr_cbk.joinable())
  {
/* Only set the priority to BG priority if the SET_THREAD_PRIORITY and */
/* ENABLE_TIMEOUT are set. If ENABLE_TIMEOUT is not set, the thread is */
/* Supposed to be finished at this point*/
#if defined(SET_THREAD_PRIORITY) && (SET_THREAD_PRIORITY == true) && \
    defined(ENABLE_TIMEOUT) && (ENABLE_TIMEOUT == true)
    if (has_timeout)
    {
      setThreadPriority(thr_cbk, DEFAULT_BG_PRIO, DEFAULT_BG_POLICY, node_name);
    }
#endif
    thr_cbk.detach();
  }
}

/**
 * @brief Get the cpu time spent by the thread thread_id
 *
 * @param thread_id Native handler to thread
 * @return std::int64_t number of microsendos spent, -1 in case of error
 */
static auto get_cpu_time_spent_us(pthread_t thread_id) -> std::int64_t
{
  clockid_t cid = 0;
  auto ret = pthread_getcpuclockid(thread_id, &cid);
  if (ret != -1)
  {
    timespec ts = {};
    ret = clock_gettime(cid, &ts);
    if (ret != -1)
    {
      // params.runtime in micro
      std::int64_t spent_us = ts.tv_sec * 1000000 + (ts.tv_nsec / 1000);
      return spent_us;
    }
    return -1;
  }
  return -1;
}

/**
 * @brief Check if the thread thread_id has spent too much time
 *
 * @param thread_id pthread handle
 * @param runtime_us expected maximum CPU time in microseconds
 * @return true has timeout
 * @return false hasn't
 */
static auto has_thread_timeout(pthread_t thread_id, int runtime_us) -> bool
{

  std::int64_t spent_us = get_cpu_time_spent_us(thread_id);
  if (spent_us == -1)
  {
    return true;
  }

  return spent_us > runtime_us;
}