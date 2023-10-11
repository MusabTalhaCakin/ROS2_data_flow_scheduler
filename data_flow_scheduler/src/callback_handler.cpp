/*******************************************************************************
 * Copyright (C) 2023 TTTech Auto AG. All rights reserved                      *
 * Operngasse 17-21, 1040 Vienna, Austria. office(at)tttech-auto.com           *
 ******************************************************************************/

#include "data_flow_scheduler/data_flow_executor.h"

#include <ctime>
#include <pthread.h>
#include <unistd.h>

using namespace DFSched;

/**
 * @brief Handles a subscription request from the Data Flow Scheduler.
 * @return True if the subscription request is handled successfully, false
 * otherwise.
 */
static auto subscription_handle(const char * /*action_description*/, const char * /*topic_or_service_name*/, std::function<bool()> /*take_action*/,
                                std::function<void()> /*handle_action*/) -> bool;

void CallbackHandler::init(const std::string &name_,
                           const TopicInfoVector callbacks_,
                           const TimerInfoVector timers_,
                           const CallbackInfoVector &callbacks)
{
  callbacks_vec = std::move(callbacks_);
  timers_vec = std::move(timers_);
  this->callbacks = callbacks;
  node_name = name_;
  size_t totalSize =
      callbacks_vec.size() + timers_vec.size() + callbacks.size();
  for (size_t i = 0; i < totalSize; ++i)
  {
    timeout_condition.push_back(std::make_unique<TimeoutConditionInfo>());
  }

  RCLCPP_INFO(rclcpp::get_logger(node_name), "CallbackHandler initialized.");
}

void CallbackHandler::run_callback(const int mtx_id, const int callback_id,
                                   const int callback_type)
{
  // std::cout << __FUNCTION__ << " id " << callback_id << " mtx_id " << mtx_id
  // << " callback type " << callback_type << std::endl;
  timeout_condition[mtx_id]->finished_.store(false);
  rclcpp::MessageInfo message_info;
  message_info.get_rmw_message_info().from_intra_process = false;

  if (3 == callback_type)
  {
    callbacks[callback_id].callback_ptr();
    timeout_condition[mtx_id]->suc_ = true;
  }
  else if (0 == callback_type)
  {
    /* Timer handle*/
    /* Should be always ready, not need to check if so */
    timers_vec[callback_id].time_ptr->execute_callback();
    timeout_condition[mtx_id]->suc_ = true;
  }
  else if (callbacks_vec[callback_id].sub_ptr != nullptr)
  {

    /* Subscription handle */
    if (callbacks_vec[callback_id].sub_ptr->is_serialized())
    {
      /* Handle serialized message */
      std::shared_ptr<rclcpp::SerializedMessage> serialized_msg =
          callbacks_vec[callback_id].sub_ptr->create_serialized_message();
      bool ret = subscription_handle(
          "taking a serialized message from topic",
          callbacks_vec[callback_id].sub_ptr->get_topic_name(),
          [&]()
          {
            return callbacks_vec[callback_id].sub_ptr->take_serialized(
                *serialized_msg.get(), message_info);
          },
          [&]()
          {
            auto void_serialized_msg =
                std::static_pointer_cast<void>(serialized_msg);
            callbacks_vec[callback_id].sub_ptr->handle_message(
                void_serialized_msg, message_info);
            timeout_condition[mtx_id]->suc_ = true;
          });

      if (!ret)
      {
        timeout_condition[mtx_id]->suc_ = false;
        RCLCPP_WARN(rclcpp::get_logger(node_name), "No data for the Callback (serialized).");
      }
    }
    else if (callbacks_vec[callback_id].sub_ptr->can_loan_messages())
    {
      /* Loaned message handle */
      void *loaned_msg = nullptr;
      subscription_handle(
          "taking a loaned message from topic",
          callbacks_vec[callback_id].sub_ptr->get_topic_name(),
          [&]()
          {
            rcl_ret_t ret = rcl_take_loaned_message(callbacks_vec[callback_id].sub_ptr->get_subscription_handle().get(),
                                                    &loaned_msg, &message_info.get_rmw_message_info(), nullptr);
            if (RCL_RET_SUBSCRIPTION_TAKE_FAILED == ret)
            {
              return false;
            }
            else if (RCL_RET_OK != ret)
            {
              rclcpp::exceptions::throw_from_rcl_error(ret);
            }
            return true;
          },
          [&]()
          {
            callbacks_vec[callback_id].sub_ptr->handle_loaned_message(
                loaned_msg, message_info);
            timeout_condition[mtx_id]->suc_ = true;
          });

      rcl_ret_t ret = rcl_return_loaned_message_from_subscription(
          callbacks_vec[callback_id].sub_ptr->get_subscription_handle().get(),
          loaned_msg);

      if (RCL_RET_OK != ret)
      {
        timeout_condition[mtx_id]->suc_ = false;
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                     "rcl_return_loaned_message_from_subscription() failed for "
                     "subscription on topic '%s': %s",
                     callbacks_vec[callback_id].sub_ptr->get_topic_name(),
                     rcl_get_error_string().str);
      }
    }
    else
    {
      /* Message handling */
      std::shared_ptr<void> message =
          callbacks_vec[callback_id].sub_ptr->create_message();
      bool ret = subscription_handle(
          "taking a message from topic",
          callbacks_vec[callback_id].sub_ptr->get_topic_name(),
          [&]()
          {
            return callbacks_vec[callback_id].sub_ptr->take_type_erased(
                message.get(), message_info);
          },
          [&]()
          {
            callbacks_vec[callback_id].sub_ptr->handle_message(message,
                                                               message_info);
            timeout_condition[mtx_id]->suc_ = true;
          });
      if (!ret)
      {
        timeout_condition[mtx_id]->suc_ = false;
        RCLCPP_WARN(rclcpp::get_logger(node_name), "No data for callback.");
      }
    }
  }

  std::lock_guard<std::mutex> lock(timeout_condition[mtx_id]->mtx_);
  timeout_condition[mtx_id]->finished_ = true;
  timeout_condition[mtx_id]->cvar_.notify_one();
}

static auto subscription_handle(const char *action_description,
                                const char *topic_or_service_name,
                                std::function<bool()> take_action,
                                std::function<void()> handle_action) -> bool
{
  bool taken = true;
  try
  {

#if defined(SUB_TAKE_MAX_TRIES) && (SUB_TAKE_MAX_TRIES > 1)
    for (size_t tries = 0; (tries <= SUB_TAKE_MAX_TRIES); tries++)
    {
      taken = take_action();
      if (taken)
      {
        break;
      }
      std::this_thread::sleep_for(
          std::chrono::microseconds(SUB_TAKE_TRY_WAIT_US));
    }
#else
    taken = take_action();
#endif
  }
  catch (const rclcpp::exceptions::RCLError &rcl_error)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "executor %s '%s' unexpectedly failed: %s", action_description,
                 topic_or_service_name, rcl_error.what());
  }
  if (taken)
  {
    handle_action();
  }
  else
  {
    // Message or Service was not taken for some reason.
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),
                 "executor %s '%s' failed to take anything", action_description,
                 topic_or_service_name);
  }
  return taken;
}