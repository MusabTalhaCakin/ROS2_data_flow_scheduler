#include "data_flow_scheduler/data_flow_sequencer.h"
#include "data_flow_scheduler/data_flow_types.h"
#include "data_flow_scheduler/graphcreator.h"
#include "data_flow_scheduler/ipc_server.h"

#include "rclcpp/parameter.hpp"

namespace DFS
{

  template <typename E>
  constexpr typename std::underlying_type<E>::type to_underlying(E e) noexcept
  {
    return static_cast<typename std::underlying_type<E>::type>(e);
  }

  /**
   * @brief ROS2 node for data flow scheduling.
   */
  class DFS : public rclcpp::Node
  {
  public:
    bool terminate = false;

    /**
     * @brief Constructor for the DFS class.
     * @param NUM_CLIENTS Number of client nodes.
     */
    DFS(int &NUM_CLIENTS)
        : Node("data_flow_scheduler"), scheduler(graph_creator)
    {
      this->declare_parameter<int>("numClients", NUM_CLIENTS);
      if (!this->get_parameter<int>("numClients", NUM_CLIENTS))
        RCLCPP_WARN(this->get_logger(),
                    "Failed to retrieve 'numClients' parameter. %d", NUM_CLIENTS);

      init(NUM_CLIENTS);
    }

    /**
     * @brief Destructor for the DFS class.
     */
    ~DFS() { std::cout << "[+]Destructor DFS!\n"; }

  private:
    int available_cores = CORES;          // Number of available cores
    DFSched::NodeInfoVector nodeinfo_vec; // Vector containing node information
    DFSched::ExecutionParams execute_;    // Execute information
    GraphCreator graph_creator;           // Graph creator object
    DFSSequencer scheduler;               // Scheduler object
    DFSServer server;                     // Server object

    /**
     * @brief Initializes the DFS class.
     * @param NUM_CLIENTS Number of client nodes.
     */
    void init(int NUM_CLIENTS)
    {

      RCLCPP_INFO(
          rclcpp::get_logger("data_flow_scheduler"),
          "INIT{ CLIENTS : %d, CORES : %d, SET_THRED_PRIORITIES : %s, LGF: %s}",
          NUM_CLIENTS, CORES, SET_THREAD_PRIORITY ? "true" : "false",
          CREATE_LGF ? "true" : "false");

      // Create socket connection
      server.build_server("data_flow_scheduler", NUM_CLIENTS);
      server.connect_clients(nodeinfo_vec, terminate, [this](int id)
                             { return this->deserialize_buffer(id); });

      if (!terminate)
      {
        // Create graph
        graph_creator.build_graph(nodeinfo_vec);
        // set true to create a cfg file from the graph
        if (CREATE_LGF)
        {
          graph_creator.save_to_file("graph.lgf"); // Set path to the destination directory
        }

        // Start scheduling
        scheduler.init("data_flow_scheduler");
        scheduler.start_sequencer(server);
      }
    }

    /**
     * @brief Deserializes the buffer received from the server.
     * @param id The ID of the client node.
     * @return Node information.
     */
    DFSched::NodeInfo deserialize_buffer(int id)
    {
      std::cout << __FUNCTION__ << std::endl;
      std::string message = server.return_buffer();

      std::istringstream iss(message);
      std::cout << "Message: " << message << "\n";
      DFSched::NodeInfo result;
      result.node_id = id;
      std::cout << "Node id " << id << "\n";

      // Extract callback topic names from the message
      std::getline(iss, message, ';');
      std::istringstream iss_topic_names(message);
      std::string topic_name;
      std::cout << " |-> Subscriptions\n";
      while (std::getline(iss_topic_names, topic_name, ','))
      {
        std::cout << " |-> " << topic_name << "\n";
        result.callback_topic_name.push_back(topic_name);
      }

      // Extract runtimes from the message
      std::getline(iss, message, ';');
      std::istringstream iss_runtimes(message);
      std::string runtime;
      std::cout << " |-> Publications\n";
      while (std::getline(iss_runtimes, runtime, ','))
      {
        result.runtime.push_back(stoi(runtime));
      }

      // Extract publisher topic names from the message
      std::getline(iss, message, ';');
      std::istringstream iss_pub_topic_names(message);
      while (std::getline(iss_pub_topic_names, topic_name, ','))
      {
        std::cout << " |-> " << topic_name << "\n";
        result.pub_topic_name.push_back(topic_name);
      }

      // Extract callback IDs from the message
      std::getline(iss, message, ';');
      std::istringstream iss_id(message);
      std::string callback_id;
      std::cout << "Callback_id\n";
      while (std::getline(iss_id, callback_id, ','))
      {
        std::cout << " |-> " << callback_id << "\n";
        result.callback_id.push_back(stoi(callback_id));
      }

      // Extract callback types from the message
      std::getline(iss, message, ';');
      std::istringstream iss_type(message);
      std::string callback_type;
      std::cout << " Callback type: \n";
      while (std::getline(iss_type, callback_type, ','))
      {
        std::cout << " |-> " << callback_type << "\n";
        result.callback_type.push_back(stoi(callback_type));
      }

      std::getline(iss, message, ';');
      std::istringstream iss_super(message);
      std::string supervision_kind;
      std::cout << " supervision kind: \n";
      while (std::getline(iss_super, supervision_kind, ','))
      {

        std::int32_t i_super = stoi(supervision_kind);
        switch (i_super)
        {
        case to_underlying(DFSched::TimeSupervision::None):
          result.supervision_kind = DFSched::TimeSupervision::None;
          break;
        case to_underlying(DFSched::TimeSupervision::ThreadCPUTime):
          result.supervision_kind = DFSched::TimeSupervision::ThreadCPUTime;
          break;
        default:
        case to_underlying(DFSched::TimeSupervision::Time):
          result.supervision_kind = DFSched::TimeSupervision::Time;
          break;
        }
      }
      std::cout << " |-> " << to_underlying(result.supervision_kind) << "\n";
      return result;
    }
  };

} // namespace DFS

int main(int argc, char **argv)
{
  int NUM_CLIENTS = (argc > 1) ? atoi(argv[1]) : 1;
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DFS::DFS>(NUM_CLIENTS);
  rclcpp::shutdown();
  return 0;
}