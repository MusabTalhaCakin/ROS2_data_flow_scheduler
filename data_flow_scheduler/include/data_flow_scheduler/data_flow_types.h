/*******************************************************************************
 * Copyright (C) 2023 TTTech Auto AG. All rights reserved                      *
 * Operngasse 17-21, 1040 Vienna, Austria. office(at)tttech-auto.com           *
 ******************************************************************************/
/**
 * @file dfstypes.h
 * @brief This file contains some define values and the data types used in the
 * Data Flow Sequencer.
 */

#ifndef DFSTYPES_H
#define DFSTYPES_H

#include <atomic>
#include <iostream>
#include <set>
#include <condition_variable>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include <stddef.h>
#include <stdexcept>
#include <stdint.h>
#include <stdio.h>

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#define SOCK_NAME "/tmp/ipc_socket.sock"

// Buffer size of the serialized data. In big system this might need to be increased.
#define BUFSIZE 1024

// How many parallel callbacks can be dispatched
static const constexpr int CORES = 2;

// Enables thread priorities for the callback dispatching
// and for the threads that are detached
#define SET_THREAD_PRIORITY false

#define DEFAULT_TASK_PRIO 1
#define DEFAULT_TASK_POLICY (SCHED_FIFO)
#define DEFAULT_BG_PRIO 0
#define DEFAULT_BG_POLICY (SCHED_OTHER)
#define DEFAULT_MONITOR_PRIO 2
#define DEFAULT_MONITOR_POLICY (SCHED_FIFO)

// This flag enables the timeout reporting to the data_flow_sequencer server
// Needs to be set true to allow setting the thread priorities
#define ENABLE_TIMEOUT false

// Specifies how many times a topic is retrived in case of failure
// or non data availability
#define SUB_TAKE_MAX_TRIES 1U
// Specifies how many microseconds waits for each try
#define SUB_TAKE_TRY_WAIT_US 100U

// How many microseconds the data_flow_sequencer waits before starting
#define SEQ_START_DELAY_MS 20000U

// Creates a Lemon Graph File of the system in the current execution path
#define CREATE_LGF true

// Enables more logs
#define VERBOSE false

// Set to true to stop the system if some node reports a callback timeout
#define THROW_IERATION_TIMEOUT false

// Set to true to stop the system if some node reports a callback execution failure
#define THROW_IERATION_EXECUTION_FAIL false

// Specifies how many traverses of the graph is performed
#define ITERATION 100000 // in microseconds

// Specifies for how long the graph is executed
#define RUNTIME 0 // in seconds, 0 for infinit loop

namespace DFS
{
  using ArcInfo = std::map<int, std::vector<int>>;
}

namespace DFSched
{

  /**
   * @brief There are 3 types of supervision:
   * \enum None Does nothing, no supervision
   * \enum ThreadCPUTime Monitors the CPU usage of the thread that dispatches the
   * activity
   * \enum Time Monitors the time spent by the task
   *
   */
  enum class TimeSupervision : std::int32_t
  {
    Time,
    None,
    ThreadCPUTime
  };

  /**
   * @brief Struct representing the information of an executed task.
   */
  struct ExecutionParams
  {
    int callb;    /**< The callback ID of the executed task. */
    int runtime;  /**< The runtime of the executed task. */
    int pr;       /**< The priority of the executed task. */
    int type;     /**< The type of the executed task. */
    bool timeout; /**< Flag indicating if the task timed out. */
    int mtx_id;   /**< The mutex ID of the executed task. */
    bool suc;     /**< The callback was executed successfully. */
    // Type of supervision that the task requests
    TimeSupervision supervision_kind;
  };

  /**
   * @brief Struct representing the information of a node.
   */
  struct NodeInfo
  {
    int node_id;                                  /**< The ID of the node. */
    std::vector<std::string> callback_topic_name; /**< The names of the callback topics. */
    std::vector<int> callback_type;               /**< The types of the callbacks. */
    std::vector<std::string> pub_topic_name;      /**< The names of the publish topics. */
    std::vector<int> runtime;                     /**< The runtimes of the callbacks. */
    std::vector<int> callback_id;                 /**< The IDs of the callbacks. */
    // Type of supervison
    TimeSupervision supervision_kind;
  };

  /**
   * @brief Struct representing the information of a callback.
   * This is used to represent a node that is subscribed to 1 topic
   * and publishes 1 topic
   */
  struct TopicInfo
  {
    std::string name;                            /**< The name of the callback. */
    int type;                                    /**< The type of the callback. */
    int runtime;                                 /**< The runtime of the callback. */
    rclcpp::SubscriptionBase::SharedPtr sub_ptr; /**< The shared pointer to the subscription. */
    int id;                                      /**< The ID of the callback. */
    std::string pub;                             /**< The name of the publish topic. */
    // Type of supervison
    TimeSupervision supervision_kind;
  };

  /**
   * @brief Struct representing the information of a timer.
   * This is used to represent a node that is triggered by a ROS2 timer and
   * produces a topic.
   */
  struct TimerInfo
  {
    std::string name;                      /**< The name of the timer. */
    int type;                              /**< The type of the timer. */
    int runtime;                           /**< The runtime of the timer. */
    rclcpp::TimerBase::SharedPtr time_ptr; /**< The shared pointer to the timer. */
    int id;                                /**< The ID of the timer. */
    std::string pub;                       /**< The name of the publish topic. */
    // Type of supervison
    TimeSupervision supervision_kind;
  };

  /**
   * @brief Struct representing the information of a timeout condition.
   */
  struct TimeoutConditionInfo
  {
    std::mutex mtx_;               /**< The mutex for the condition variable. */
    std::condition_variable cvar_; /**< The condition variable for the timeout. */
    std::atomic<bool> finished_;   /**< Flag indicating if the timeout condition has finished. */
    bool suc_;                     /**< Flag indicating if the timeout condition was successful. */

    TimeoutConditionInfo() = default;

    // Delete copy operations
    TimeoutConditionInfo(const TimeoutConditionInfo &) = delete;
    TimeoutConditionInfo &operator=(const TimeoutConditionInfo &) = delete;

    // Allow move operations
    TimeoutConditionInfo(TimeoutConditionInfo &&) = default;
    TimeoutConditionInfo &operator=(TimeoutConditionInfo &&) = default;
  };

  inline void setThreadPriority(std::thread &threadObj, int priority, int policy,
                                const std::string &node_name)
  {
    // Get the native handle of the thread
    pthread_t nativeHandle = threadObj.native_handle();

    // Create a scheduling parameters object
    sched_param param;
    param.sched_priority = priority;

    // Set the scheduling parameters for the native handle
    if (pthread_setschedparam(nativeHandle, policy, &param) != 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger(node_name + "|ACTIVITY"),
                   "Failed to set thread priority.");
      // Handle the error
    }
  }

  /**
   * @brief Struct representing the information of a callback.
   * This is used to represent a node that is subscribed to multiple topics
   * and produces multiple topics.
   * Functor callback_ptr is called when all the subscribed topics are ready
   */
  struct CallbackInfo
  {
    std::vector<std::string> subs;      /**< List of topics subscribed. */
    int type = 3;                       /**< The type of the callback. */
    int runtime;                        /**< The runtime of the callback. */
    std::function<void()> callback_ptr; /**< Callback function pointer the subscription. */
    int id;                             /**< The ID of the callback. */
    std::vector<std::string> pubs;      /**< List of publish topics. */
    // Type of supervison
    TimeSupervision supervision_kind;
  };

  using CallbackInfoVector = std::vector<DFSched::CallbackInfo>;
  using NodeInfoVector = std::vector<DFSched::NodeInfo>;
  using TopicInfoVector = std::vector<DFSched::TopicInfo>;
  using TimerInfoVector = std::vector<DFSched::TimerInfo>;
  using TimeoutInfoVector = std::vector<std::unique_ptr<TimeoutConditionInfo>>;

} // namespace DFSched

#endif