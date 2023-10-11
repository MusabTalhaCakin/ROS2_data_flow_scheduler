/*******************************************************************************
 * Copyright (C) 2023 TTTech Auto AG. All rights reserved                      *
 * Operngasse 17-21, 1040 Vienna, Austria. office(at)tttech-auto.com           *
 ******************************************************************************/

#ifndef DFS_SERVER_H
#define DFS_SERVER_H

#include "data_flow_scheduler/data_flow_types.h"

namespace DFS {
/**
 * @class DFSServer
 * @brief The DFSServer class handles the server-side functionality of the Data
 * Flow Scheduler.
 */
class DFSServer {
public:
  /**
   * @brief Default constructor for the DFSServer class.
   */
  DFSServer() : server_sock(-1){};

  /**
   * @brief Builds the server by specifying the address and port to bind.
   * @param address The address to bind.
   * @param port The port to bind.
   */
  void build_server(const std::string &, const int);

  /**
   * @brief Sets the file descriptor for monitoring server sockets.
   */
  void set_fd();

  /**
   * @brief Sends raw data through the specified socket.
   * @param socket The socket to send the data.
   * @param data Pointer to the data to send.
   * @param size The size of the data to send.
   * @return True if the data is sent successfully, false otherwise.
   */
  bool send_raw_data(int, const void *, std::uint32_t) const;

  /**
   * @brief Reads raw data from the specified socket.
   * @param socket The socket to read the data.
   * @param buffer Pointer to the buffer to store the received data.
   * @param size The size of the buffer.
   * @return The number of bytes read, or -1 if an error occurs.
   */
  int read_raw_data(int, void *, std::uint32_t) const;

  /**
   * @brief Establishes connections with the clients.
   * @param nodeinfo_vec Vector of NodeInfo objects.
   * @param terminate True if termination occures.
   * @param deserialization_function Deserializes the buffer and stores them in
   * a node_info object vector.
   */
  void connect_clients(DFSched::NodeInfoVector &, bool &,
                       const std::function<DFSched::NodeInfo(int)> &);

  /**
   * @brief Handles the response received from the clients.
   * @param execute_info Reference to the Execute_Info object to store the
   * execution information.
   * @param available_cores Reference to the variable cores.
   * @param executed Stores all executed Callbacks.
   * @param executed_last Stores the executed Callbacks from the last iteration.
   * @return True if the response is successfully handled, false otherwise.
   */
  bool handle_response(DFSched::ExecutionParams &, int &, std::vector<int> &,
                       std::vector<int> &);

  /**
   * @brief Destructor for the DFSServer class.
   */
  ~DFSServer() {
    close_connections();
    std::cout << "[+]Destructor DFSServer!\n";
  };

  /**
   * @brief Returns the buffer as a string.
   * @return The buffer content as a string.
   */
  std::string return_buffer() { return std::string(buffer); }

  /**
   * @brief Signal handler function.
   * @param signal The signal received.
   */
  static void handleSignal(int) { signalReceived = 1; }

  /**
   * @brief Sets the signal handler for handling signals.
   */
  void setSignalHandler();

private:
  static volatile sig_atomic_t
      signalReceived;    /**< Indicates whether a signal has been received. */
  int num_clients;       /**< The number of connected clients. */
  std::string node_name; /**< The name of the node. */
  int server_sock;       /**< The server socket file descriptor. */
  std::vector<int>
      client_sock;         /**< Vector of client socket file descriptors. */
  char buffer[BUFSIZE];    /**< The buffer for storing data. */
  struct sockaddr_un addr; /**< The address for binding the server socket. */
  fd_set readfds;          /**< The set of file descriptors for monitoring. */
  int max_sd, act_; /**< Variables for tracking the maximum file descriptor and
                       activity. */

  /**
   * @brief Binds the server socket to the specified address and port.
   * @return True if the server socket is successfully bound, false otherwise.
   */
  bool bind_socket();

  /**
   * @brief Closes all client connections and the server socket.
   */
  void close_connections();

  /**
   * @brief Creates the server socket.
   * @return True if the server socket is created successfully, false otherwise.
   */
  bool create_socket();

  /**
   * @brief Initializes the address for binding the server socket.
   */
  void init_adress();
};

} // namespace DFS

#endif