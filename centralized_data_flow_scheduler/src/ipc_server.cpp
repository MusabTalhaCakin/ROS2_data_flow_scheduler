#include "centralized_data_flow_scheduler/ipc_server.h"

using namespace DFS;

volatile sig_atomic_t DFSServer::signalReceived = 0;

bool DFSServer::create_socket()
{
  // Unlink the socket to clear any previous instances
  unlink(SOCK_NAME);

  // Creating the socket
  server_sock = socket(AF_UNIX, SOCK_STREAM, 0);
  if (server_sock < 0)
  {
    return false;
  }

  RCLCPP_INFO(rclcpp::get_logger(node_name), "Socket Created.");
  return true;
}

void DFSServer::init_adress()
{
  // Initialize the address structure
  memset(&addr, 0, sizeof(addr));
  addr.sun_family = AF_UNIX;
  strncpy(addr.sun_path, SOCK_NAME, sizeof(addr.sun_path) - 1);
}

bool DFSServer::bind_socket()
{
  // Binding the socket to the address
  if (bind(server_sock, (struct sockaddr *)&addr, sizeof(addr)) < 0)
  {
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger(node_name), "Bind socket.");
  return true;
}

void DFSServer::close_connections()
{
  // Close connection
  for (int i = 0; i < num_clients; i++)
  {
    close(client_sock[i]);
  }
  RCLCPP_INFO(rclcpp::get_logger(node_name), "Close sockets.");
}

void DFSServer::set_fd()
{

  FD_ZERO(&readfds);
  FD_SET(server_sock, &readfds);
  max_sd = server_sock;

  for (int i = 0; i < num_clients; i++)
  {
    FD_SET(client_sock[i], &readfds);
    if (client_sock[i] > max_sd)
    {
      max_sd = client_sock[i];
    }
  }
}

void DFSServer::build_server(const std::string &name_, const int num_)
{
  node_name = name_;
  num_clients = num_;
  create_socket();
  init_adress();
  bind_socket();
}

void DFSServer::connect_clients(DFS_Interface::NodeInfoVector &nodeinfo_vec,
                                bool &terminate,
                                const std::function<DFS_Interface::Node_Info(int)> &deserialize_buffer)
{
  setSignalHandler();
  client_sock.resize(num_clients);
  // Listen for incoming connections
  if (listen(server_sock, num_clients) < 0)
  {
    perror("[-]Error listening");
    RCLCPP_ERROR(rclcpp::get_logger(node_name + "|DFSServer"), "Error listening");
    std::exit(EXIT_FAILURE);
  }
  std::cout << " Listen ...\n";

  // Accept connections from multiple clients
  for (int i = 0; i < num_clients; i++)
  {
    client_sock[i] = accept(server_sock, NULL, NULL);
    if (signalReceived)
    {
      terminate = true;
      break;
    }
    read(client_sock[i], &buffer, BUFSIZE);
    DFS_Interface::Node_Info nodeinfo_ = deserialize_buffer(i);
    std::cout << "[" << i << "] BUFFER:" << buffer << std::endl;

    nodeinfo_vec.push_back(nodeinfo_);
  }

  if (signalReceived)
    RCLCPP_INFO(rclcpp::get_logger(node_name), "Terminate DFSServer.");

  else
  {
    RCLCPP_INFO(rclcpp::get_logger(node_name), "All clients connected.");
  }
}

bool DFSServer::send_raw_data(int id, const void *data, std::uint32_t data_size) const
{
  auto ret = write(client_sock[id], data, data_size);
  return ret != -1;
}

int DFSServer::read_raw_data(int id, void *data, std::uint32_t data_size) const
{
  auto ret = read(client_sock[id], data, data_size);
  return ret;
}

bool DFSServer::handle_response(DFS_Interface::Execute_Info &execute_,
                                int &available_cores,
                                std::vector<int> &executed,
                                std::vector<int> &executed_last_iteration)
{
  setSignalHandler();
  act_ = select(max_sd + 1, &readfds, NULL, NULL, NULL);
  for (int k = 0; k < num_clients; k++)
  {
    if (signalReceived)
    {
      return false;
    }
    if (FD_ISSET(client_sock[k], &readfds))
    {
      read(client_sock[k], &execute_, sizeof(DFS_Interface::Execute_Info));
      available_cores++;
      if (VERBOSE)
        RCLCPP_INFO(rclcpp::get_logger(node_name),
                    "READ[%d] -> ID:%d | Timeout:%s",
                    k, execute_.callb, execute_.timeout ? "true" : "false");
      executed.push_back(execute_.pr);
      executed_last_iteration.push_back(execute_.pr);
    }
  }
  return true;
}

void DFSServer::setSignalHandler()
{
  struct sigaction sa;
  sa.sa_handler = handleSignal;
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = 0;
  sigaction(SIGINT, &sa, nullptr);
}