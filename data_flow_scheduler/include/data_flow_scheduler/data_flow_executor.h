/*******************************************************************************
 * Copyright (C) 2023 TTTech Auto AG. All rights reserved                      *
 * Operngasse 17-21, 1040 Vienna, Austria. office(at)tttech-auto.com           *
 ******************************************************************************/

#ifndef DFS_EXECUTOR_H
#define DFS_EXECUTOR_H

#include "data_flow_scheduler/callback_handler.h"
#include "data_flow_scheduler/data_flow_types.h"
#include "data_flow_scheduler/ipc_client.h"

#include <thread>
#include <vector>

#include <pthread.h>

namespace DFSched {

/**
 * @class DFSExecutor
 * @brief This class represents an DFSExecutor in the Data Flow Scheduler.
 *
 * A DFSExecutor contains units of executables that can be scheduled and
 * executed by the Data Flow Scheduler. It can have both timer-based and
 * topic-based callbacks.
 */
class DFSExecutor {
public:
  /**
   * @brief Constructs an DFSExecutor with timer-based executions.
   * @param node_name The name of the DFSExecutor node.
   * @param timers A vector containing timer informations for the DFSExecutor.
   */
  DFSExecutor(const std::string, TimerInfoVector);

  /**
   * @brief Constructs an DFSExecutor with topic-based executions.
   * @param node_name The name of the DFSExecutor node.
   * @param callbacks A vector containing topic informations for the
   * DFSExecutor.
   */
  DFSExecutor(const std::string, TopicInfoVector);

  /**
   * @brief Constructs an DFSExecutor with both timer-based and callback-based
   * executions.
   * @param node_name The name of the DFSExecutor node.
   * @param timers A vector containing timer information for the DFSExecutor.
   * @param callbacks A vector containing topic informations for the
   * DFSExecutor.
   */
  DFSExecutor(const std::string, TimerInfoVector, TopicInfoVector);

  DFSExecutor(const std::string &, CallbackInfoVector);

  /**
   * @brief Destructor for the DFSExecutor class.
   */
  ~DFSExecutor() {
    std::cout << "[+]Destructor DFSched::DFSExecutor\n";
    for (auto th : detached_threads) {
      if (th != nullptr) {
        th->join();
        delete th;
      }
    }
  };

  /**
   * @brief Spins the DFSExecutor for execution.
   */
  void spin();

private:
  std::string node_name; /**< The name of the node. */
  NodeInfo node_info;    /**< Information about the DFSExecutor node. */
  DFSched::CallbackHandler
      cb_handler;   /**< Handles the execution of callbacks. */
  DFSClient client; /**< Client generator to communicate with the data flow
                       scheduler. */
  TopicInfoVector topics_; /**< Topic informations for the DFSExecutor. */
  TimerInfoVector timers; /**< Timer informations for the DFSExecutor. */
  CallbackInfoVector callbacks;
  std::vector<std::thread *>
      detached_threads; /**< Holds the threads that has been detached */

  /**
   * @brief Initiates the DFSExecutor and establishes communication with the
   * data flow scheduler.
   * @details This function sets up the necessary configurations and
   * communication channels to interact with the Data Flow Scheduler.
   */
  void init();

  /**
   * @brief Sends callback informations to the data flow scheduler.
   * @return True if the information is sent successfully, false otherwise.
   */
  bool send_callback_infos();

  /**
   * @brief Serializes the buffer for communication.
   * @return The serialized buffer.
   */
  std::string serialize_buffer() const;

  /**
   * @brief Reads and processes messages received from the data flow scheduler.
   * @param info Information about the execution.
   */
  void read_msgs(ExecutionParams);
};

} // namespace DFSched

#endif