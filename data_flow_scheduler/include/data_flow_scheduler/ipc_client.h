/*******************************************************************************
 * Copyright (C) 2023 TTTech Auto AG. All rights reserved                      *
 * Operngasse 17-21, 1040 Vienna, Austria. office(at)tttech-auto.com           *
 ******************************************************************************/

#ifndef DFS_CLIENT_H
#define DFS_CLIENT_H

#include "data_flow_scheduler/data_flow_types.h"

namespace DFS_Interface
{

  /**
   * @class DFSClient
   * @brief This class represents a client for the Data Flow Scheduler.
   *
   * The client is responsible for establishing a connection with the sercer and
   * exchanging data and messages.
   */
  class DFSClient
  {
  public:
    /**
     * @brief Constructs a DFSClient object.
     */
    DFSClient() : sock(-1){};

    /**
     * @brief Destructor for the DFSClient class.
     * @details It performs necessary clean-up when a DFSClient object is destroyed.
     */
    ~DFSClient()
    {
      close_socket();
      std::cout << "[+]Destructor DFSClient!\n";
    };

    /**
     * @brief Connects the client to the Data Flow Scheduler server socket.
     * @param node_name The name of the node.
     * @return True if the connection is established successfully, false otherwise.
     */
    bool connect(const std::string &);

    /**
     * @brief Sends data to the Data Flow Scheduler.
     * @param data The data to be sent.
     * @return True if the data is sent successfully, false otherwise.
     */
    bool send_data(const std::string &);

    /**
     * @brief Sends raw data to the Data Flow Scheduler.
     * @param data The raw data to be sent.
     * @param size The size of the data in bytes.
     * @return True if the raw data is sent successfully, false otherwise.
     */
    bool send_raw_data(const void *,
                       std::uint32_t) const;

    /**
     * @brief Reads raw data from the Data Flow Scheduler.
     * @param buffer The buffer to store the received data.
     * @param size The size of the buffer in bytes.
     * @return The number of bytes read, or -1 if an error occurs.
     */
    int read_raw_data(void *,
                      std::uint32_t) const;

  private:
    std::string node_name;   /**< The name of the node. */
    int sock;                /**< The socket for communication. */
    struct sockaddr_un addr; /**< The address for connecting to the platform. */
    char buffer[BUFSIZE];    /**< The buffer for data exchange. */

    /**
     * @brief Closes the socket used for communication.
     */
    void close_socket();
  };

} // namespace DFS_Interface

#endif