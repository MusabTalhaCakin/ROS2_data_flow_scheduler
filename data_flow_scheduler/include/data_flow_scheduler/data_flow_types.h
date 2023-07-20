/*******************************************************************************
 * Copyright (C) 2023 TTTech Auto AG. All rights reserved                      *
 * Operngasse 17-21, 1040 Vienna, Austria. office(at)tttech-auto.com           *
 ******************************************************************************/
/**
 * @file dfstypes.h
 * @brief This file contains some define values and the data types used in the Data Flow Sequencer.
 */

#ifndef DFSTYPES_H
#define DFSTYPES_H
#include <atomic>
#include <stdio.h>
#include <stddef.h>
#include <stdint.h>
#include <stdexcept>
#include <iostream>
#include <set>

#include <string>
#include <vector>
#include <mutex>
#include <condition_variable>

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"

#define SOCK_NAME "/tmp/ipc_socket.sock"
#define BUFSIZE 1024
<<<<<<< Updated upstream
#define CORES 3
#define SET_THREAD_PRIORITY false
#define CREATE_LGF false
#define VERBOSE false
=======
#define CORES 1
#define SET_THREAD_PRIORITY false
#define CREATE_LGF false
#define VERBOSE false
#define THROW_IERATION false
>>>>>>> Stashed changes
#define ITERATION 100000 // in microseconds
#define RUNTIME 60       // in seconds

namespace DFS_Interface
{

  /**
   * @brief Struct representing the information of an executed task.
   */
  struct Execute_Info
  {
    int callb;    /**< The callback ID of the executed task. */
    int runtime;  /**< The runtime of the executed task. */
    int pr;       /**< The priority of the executed task. */
    int type;     /**< The type of the executed task. */
    bool timeout; /**< Flag indicating if the task timed out. */
    int mtx_id;   /**< The mutex ID of the executed task. */
    bool suc;     /**< The callback was executed successfully. */  
  };

  /**
   * @brief Struct representing the information of a node.
   */
  struct Node_Info
  {
    int node_id;                                  /**< The ID of the node. */
    std::vector<std::string> callback_topic_name; /**< The names of the callback topics. */
    std::vector<int> callback_type;               /**< The types of the callbacks. */
    std::vector<std::string> pub_topic_name;      /**< The names of the publish topics. */
    std::vector<int> runtime;                     /**< The runtimes of the callbacks. */
    std::vector<int> callback_id;                 /**< The IDs of the callbacks. */
  };

  /**
   * @brief Struct representing the information of a callback.
   */
  struct Topic_Info
  {
    std::string name;                            /**< The name of the callback. */
    int type;                                    /**< The type of the callback. */
    int runtime;                                 /**< The runtime of the callback. */
    rclcpp::SubscriptionBase::SharedPtr sub_ptr; /**< The shared pointer to the subscription. */
    int id;                                      /**< The ID of the callback. */
    std::string pub;                             /**< The name of the publish topic. */
  };

  /**
   * @brief Struct representing the information of a timer.
   */
  struct Timer_Info
  {
    std::string name;                      /**< The name of the timer. */
    int type;                              /**< The type of the timer. */
    int runtime;                           /**< The runtime of the timer. */
    rclcpp::TimerBase::SharedPtr time_ptr; /**< The shared pointer to the timer. */
    int id;                                /**< The ID of the timer. */
    std::string pub;                       /**< The name of the publish topic. */
  };

  /**
   * @brief Struct representing the information of a timeout condition.
   */
  struct Timeout_Condition_Info
  {
    std::mutex mtx_;               /**< The mutex for the condition variable. */
    std::condition_variable cvar_; /**< The condition variable for the timeout. */
    std::atomic<bool> finished_;   /**< Flag indicating if the timeout condition has finished. */
    bool suc_;                     /**< Flag indicating if the timeout condition was successful. */

    Timeout_Condition_Info() = default;

    // Delete copy operations
    Timeout_Condition_Info(const Timeout_Condition_Info &) = delete;
    Timeout_Condition_Info &operator=(const Timeout_Condition_Info &) = delete;

    // Allow move operations
    Timeout_Condition_Info(Timeout_Condition_Info &&) = default;
    Timeout_Condition_Info &operator=(Timeout_Condition_Info &&) = default;
  };

  inline void setThreadPriority(std::thread &threadObj,
                                int priority, const std::string &node_name)
  {
    // Get the native handle of the thread
    pthread_t nativeHandle = threadObj.native_handle();

    // Create a scheduling parameters object
    sched_param param;
    param.sched_priority = priority;

    // Set the scheduling parameters for the native handle
    if (pthread_setschedparam(nativeHandle, SCHED_FIFO, &param) != 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger(node_name + "|ACTIVITY"), "Failed to set thread priority.");
      // Handle the error
    }
  }

  using NodeInfoVector = std::vector<DFS_Interface::Node_Info>;
  using TopicInfoVector = std::vector<DFS_Interface::Topic_Info>;
  using TimerInfoVector = std::vector<DFS_Interface::Timer_Info>;
  using TimeoutInfoVector = std::vector<std::unique_ptr<Timeout_Condition_Info>>;

} // namespace DFS_Interface

#endif