/*******************************************************************************
 * Copyright (C) 2023 TTTech Auto AG. All rights reserved                      *
 * Operngasse 17-21, 1040 Vienna, Austria. office(at)tttech-auto.com           *
 ******************************************************************************/

#include "data_flow_scheduler/ipc_client.h"

using namespace DFSched;

bool DFSClient::connect(const std::string &name_)
{
  // Create a socket
  node_name = name_;
  sock = socket(AF_UNIX, SOCK_STREAM, 0);
  if (sock < 0)
  {
    return false;
  }
  memset(&addr, 0, sizeof(addr));
  addr.sun_family = AF_UNIX;
  strncpy(addr.sun_path, SOCK_NAME, sizeof(addr.sun_path) - 1);
  RCLCPP_INFO(rclcpp::get_logger(node_name), "Creating socket.");

  // Connect to the server
  if (::connect(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0)
  {
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger(node_name), "Connect to server.");

  return true;
}

void DFSClient::close_socket()
{
  if (sock != -1)
  {
    close(sock);
    sock = -1;
  }
  RCLCPP_INFO(rclcpp::get_logger(node_name), "Close sockets.");
}

bool DFSClient::send_data(const std::string &data)
{
  // Copy the data into the buffer and send it
  strncpy(buffer, data.c_str(), sizeof(buffer));
  auto n = write(sock, buffer, strlen(buffer) + 1);
  if (n == -1)
  {
    RCLCPP_ERROR(rclcpp::get_logger(node_name),
                 "Something went wrong sending activity\n");
    return false;
  }

  // RCLCPP_INFO(rclcpp::get_logger(node_name), "Send Activity info to DFS.\n");

  return true;
}

bool DFSClient::send_raw_data(const void *data, std::uint32_t data_size) const
{
  // Send raw data over the socket
  auto ret = write(sock, data, data_size);
  return ret != -1;
}

int DFSClient::read_raw_data(void *data, std::uint32_t data_size) const
{
  // Read raw data from the socket
  auto ret = read(sock, data, data_size);

  if (ret == -1)
  {
    RCLCPP_ERROR(rclcpp::get_logger(node_name), "Server DISCONNECTED!");
  }
  // std::string strdata((const char *)data);
  //  RCLCPP_INFO(rclcpp::get_logger(node_name),
  //              "READED: '%s' total bytes: %d expected %d", strdata.c_str(),
  //              ret, data_size);

  return ret;
}