/*******************************************************************************
 * Copyright (C) 2023 TTTech Auto AG. All rights reserved                      *
 * Operngasse 17-21, 1040 Vienna, Austria. office(at)tttech-auto.com           *
 ******************************************************************************/

#ifndef CALLBACK_HANDLER_H
#define CALLBACK_HANDLER_H

#include "centralized_data_flow_scheduler/data_flow_types.h"

namespace DFS_Interface
{

  /**
   * @class CallbackHandler
   * @brief This class is responsible for the execution of the callbacks.
   *
   * It handles the execution of callbacks based on their ID and type arguments.
   */
  class CallbackHandler
  {
  public:
    TimeoutInfoVector timeout_condition; /**< Vector of timeout conditions. */

    /**
     * @brief Constructs a CallbackHandler object.
     */
    CallbackHandler(){};

    /**
     * @brief Destructor for the CallbackHandler class.
     */
    ~CallbackHandler()
    {
      std::cout << "[+]Destructor PAS::CallbackHandler\n";
    };

    /**
     * @brief Executes the specified callback.
     * @param mutex_id The ID of the mutex associated with the callback.
     * @param callback_id The ID of the callback.
     * @param callback_type The type of the callback.
     */
    void run_callback(const int, const int, const int);

    /**
     * @brief Initializes the CallbackHandler with necessary information.
     * @param node_name Name of the node associated with the dfsexecutor.
     * @param callbacks Vector of callback information.
     * @param timers Vector of timer information.
     */
    void init(const std::string, const TopicInfoVector,
              const TimerInfoVector);

    /**
     * @brief Handles a subscription request from the Centralized Data Flow Scheduler.
     * @return True if the subscription request is handled successfully, false otherwise.
     */
    bool subscription_handle(const char *, const char *,
                             std::function<bool()>,
                             std::function<void()>);

  private:
    std::string node_name;         /**< The name of the node associated with the dfsexecutor. */
    TopicInfoVector callbacks_vec; /**< The vector of callback information. */
    TimerInfoVector timers_vec;    /**< The vector of timer information. */
  };

} // namespace DFS_Interface

#endif