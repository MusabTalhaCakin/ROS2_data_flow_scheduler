/*******************************************************************************
 * Copyright (C) 2023 TTTech Auto AG. All rights reserved                      *
 * Operngasse 17-21, 1040 Vienna, Austria. office(at)tttech-auto.com           *
 ******************************************************************************/

#ifndef GRAPHCREATOR_H
#define GRAPHCREATOR_H

#include "data_flow_scheduler/data_flow_types.h"
#include "data_flow_scheduler/igraphcreator.h"

// lemon
#include <lemon/list_graph.h>
#include <lemon/lgf_writer.h>
#include <lemon/graph_to_eps.h>
#include <lemon/math.h>
#include <lemon/bellman_ford.h>

namespace DFS
{

  /**
   * @class GraphCreator
   * @brief A class responsible for creating and manipulating graphs.
   */
  class GraphCreator : public IGraphCreator
  {
  public:
    /**
     * @brief Constructor for GraphCreator.
     */
    GraphCreator();
    /**
     * @brief Destructor for GraphCreator.
     */
    ~GraphCreator()
    {
      std::cout << "[+]Destructor PAS::GraphCreator\n";
    };

    /**
     * @brief Builds the graph based on the provided node information.
     * @param nodeinfo_vec The vector containing node information.
     */
    void build_graph(const DFS_Interface::NodeInfoVector &) override;

    /**
     * @brief Saves the graph to a file in the LGF format.
     * @param path The path to the output file.
     */
    void save_to_file(const std::string &) const override;

    /**
     * @brief Retrieves the underlying graph.
     * @return A reference to the underlying lemon::ListDigraph.
     */
    lemon::ListDigraph &get_graph() override
    {
      return graph_;
    }

    /**
     * @brief Retrieves the longest path value for a given node.
     * @param pr The node identifier.
     * @return The longest path value.
     */
    int get_longestpath(const int pr) const override
    {
      return (*longest_path)[graph_.nodeFromId(pr)];
    }

    /**
     * @brief Retrieves the node ID for a given node.
     * @param pr The node identifier.
     * @return The node ID.
     */
    int get_node_id(const int pr) const override
    {
      return (*node_id)[graph_.nodeFromId(pr)];
    }

    int get_seq_num(const int pr) const override
    {
      return (*seq_num)[graph_.nodeFromId(pr)];
    }

    void set_seq_num(const int pr) const override
    {
      (*seq_num)[graph_.nodeFromId(pr)] += 1;
    }

    std::string get_node_name(const int pr) const override
    {
      return (*node_name)[graph_.nodeFromId(pr)];
    }

    std::string get_sub_name(const int pr) const override
    {
      return (*sub)[graph_.nodeFromId(pr)];
    }

    /**
     * @brief Retrieves the callback ID for a given node.
     * @param pr The node identifier.
     * @return The callback ID.
     */
    int get_callback_id(const int pr) const override
    {
      return (*callback_id)[graph_.nodeFromId(pr)];
    }

    /**
     * @brief Retrieves the callback type for a given node.
     * @param pr The node identifier.
     * @return The callback type.
     */
    int get_callback_type(const int pr) const override
    {
      return (*callback_type)[graph_.nodeFromId(pr)];
    }

    /**
     * @brief Retrieves the mutex ID for a given node.
     * @param pr The node identifier.
     * @return The mutex ID.
     */
    int get_mutex_id(const int pr) const override
    {
      return (*mutex_id)[graph_.nodeFromId(pr)];
    }

    /**
     * @brief Retrieves the runtime for a given node.
     * @param pr The node identifier.
     * @return The runtime.
     */
    int get_runtime(const int pr) const override
    {
      return (*runtime)[graph_.nodeFromId(pr)];
    }

  private:
    lemon::ListDigraph graph_;                                           /**< The underlying graph. */
    std::unique_ptr<lemon::ListDigraph::NodeMap<int>> seq_num;           /**< Seq num map. */
    std::unique_ptr<lemon::ListDigraph::NodeMap<int>> node_id;           /**< Node ID map. */
    std::unique_ptr<lemon::ListDigraph::NodeMap<std::string>> node_name; /**< Node Name map. */
    std::unique_ptr<lemon::ListDigraph::NodeMap<std::string>> sub;       /**< Subtopic map. */
    std::unique_ptr<lemon::ListDigraph::NodeMap<std::string>> pub;       /**< Pubtopic map. */
    std::unique_ptr<lemon::ListDigraph::NodeMap<int>> runtime;           /**< Runtime map. */
    std::unique_ptr<lemon::ListDigraph::NodeMap<int>> longest_path;      /**< Longest path map. */
    std::unique_ptr<lemon::ListDigraph::NodeMap<int>> callback_id;       /**< Callback ID map. */
    std::unique_ptr<lemon::ListDigraph::NodeMap<int>> callback_type;     /**< Callback type map. */
    std::unique_ptr<lemon::ListDigraph::NodeMap<int>> mutex_id;          /**< Mutex ID map. */
    std::unique_ptr<lemon::ListDigraph::ArcMap<int>> run_time;           /**< Runtime for each arc. */

    /**
     * @brief Computes the longest path in the graph using the Bellman-Ford algorithm.
     */
    void compute_longestpath();
  };

} // namespace DFS

#endif