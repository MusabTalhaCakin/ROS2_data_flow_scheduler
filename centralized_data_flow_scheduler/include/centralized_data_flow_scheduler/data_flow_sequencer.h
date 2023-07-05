/*******************************************************************************
 * Copyright (C) 2023 TTTech Auto AG. All rights reserved                      *
 * Operngasse 17-21, 1040 Vienna, Austria. office(at)tttech-auto.com           *
 ******************************************************************************/

#ifndef DFS_SEQUENCER_H
#define DFS_SEQUENCER_H

#include "centralized_data_flow_scheduler/data_flow_types.h"
#include "centralized_data_flow_scheduler/data_flow_hashT.h"
#include "centralized_data_flow_scheduler/igraphcreator.h"
#include "centralized_data_flow_scheduler/ipc_server.h"

namespace DFS
{

  class DFSSequencer
  {
  public:
    /**
     * @brief Constructs a DFSSequencer object with the specified graph creator.
     * @param graph_creator The graph creator object.
     */
    DFSSequencer(IGraphCreator &graph_creator)
        : gcreator(graph_creator){};

    /**
     * @brief Initializes the sequencer with the specified node name.
     * @param node_name The name of the node.
     */
    void init(const std::string &);

    /**
     * @brief Starts the sequencing process using the specified server.
     */
    void start_sequencer(DFSServer &);

    /**
     * @brief Destructor for the DFSSequencer object.
     */
    ~DFSSequencer()
    {
      std::cout << "[+]Destructor DFSSequencer!\n";
    };

    /**
     * @brief Signal handler for catching interrupt signals.
     * @param signal The received signal.
     */
    static void handleSignal(int)
    {
      signalReceived = 1;
    }

    /**
     * @brief Sets the signal handler to catch interrupt signals.
     */
    void setSignalHandler()
    {
      struct sigaction sa;
      sa.sa_handler = handleSignal;
      sigemptyset(&sa.sa_mask);
      sa.sa_flags = 0;
      sigaction(SIGINT, &sa, nullptr);
    }

  private:
    static volatile sig_atomic_t signalReceived; /**< Indicates whether a signal has been received. */
    DFSHashT hashtables;                         /**< The hash tables for storing execution information. */
    std::string node_name;                       /**< The name of the node. */
    IGraphCreator &gcreator;                     /**< The graph creator object. */
    DFS_Interface::Execute_Info execute_;        /**< The execution information. */
    std::vector<int> ready_list;                 /**< The list of ready callbacks for execution. */
    std::vector<int> executed;                   /**< The list of executed callbacks. */
    std::vector<int> executed_last;              /**< The list of executed callbacks in the last iteration. */
    std::vector<int> runtime_count;              /**< Counts the runtimes of each iteration. */
    bool first;                                  /**< Indicates whether it is the first iteration. */
    int available_cores = CORES;                 /**< The number of available cores for execution. */

    /**
     * @brief Prints the elements of the given vector.
     * @param vec The vector to be printed.
     */
    void print_vector(const std::vector<int> &) const;

    /**
     * @brief Prints the (max, min and avarage) of runtime_count.
     */
    void print_runtime() const;

    /**
     * @brief Checks if all dependent callbacks of a given callback have been executed.
     * @param InArcs The hash table with the incoming arc dependancies.
     * @param OutArcs The hash table with the outgoing arc dependancies.
     */
    void all_dependet_nodes_executed(std::map<int, std::vector<int>> &,
                                     std::map<int, std::vector<int>>);

    /**
     * @brief Sequencer to handle the created graph.
     * @param node_id The ID of the node.
     * @param graph_node_id the ID of the graph node.
     * @param readylist The Vector which contains the executable callback ID's
     * @param callback_id The ID of the callback function.
     * @param callback_type The type of the callback function.
     * @param executed_last The list of executed callback in the last iteration.
     * @param mutex_id The ID of the mutex.
     * @param server The PASServer object.
     */
    void execute_callback(int node_id, int callback_id, int callback_type,
                          std::vector<int> &executed_last,
                          int mutex_id, int callback_nodes, int callback_cores,
                          DFSServer &server);
  };

} // namespace DFS

#endif
