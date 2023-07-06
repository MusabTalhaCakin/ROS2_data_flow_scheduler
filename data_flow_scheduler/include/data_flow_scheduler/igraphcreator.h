/*******************************************************************************
 * Copyright (C) 2023 TTTech Auto AG. All rights reserved                      *
 * Operngasse 17-21, 1040 Vienna, Austria. office(at)tttech-auto.com           *
 ******************************************************************************/

#ifndef IGRAPHCREATOR_H
#define IGRAPHCREATOR_H

#include "data_flow_scheduler/data_flow_types.h"
#include <lemon/list_graph.h>

namespace DFS
{

  /**
   * @class IGraphCreator
   * @brief Interface class for graph creation in the Platform Activity Sequencer.
   */
  class IGraphCreator
  {
  public:
    /**
     * Builds the graph based on the given node information vector.
     * @param node_info_vector The vector containing node information.
     */
    virtual void build_graph(const DFS_Interface::NodeInfoVector &node_info_vector) = 0;

    /**
     * Saves the graph to a file.
     * @param filename The name of the file to save the graph to.
     */
    virtual void save_to_file(const std::string &filename) const = 0;

    /**
     * Returns a reference to the underlying graph.
     * @return Reference to the underlying graph.
     */
    virtual lemon::ListDigraph &get_graph() = 0;

    /**
     * Returns the longest path value for the given node.
     * @param pr The node identifier.
     * @return The longest path value.
     */
    virtual int get_longestpath(const int pr) const = 0;

    /**
     * Returns the node ID for the given node.
     * @param pr The node identifier.
     * @return The node ID.
     */
    virtual int get_node_id(const int pr) const = 0;

    /**
     * Returns the callback ID for the given node.
     * @param pr The node identifier.
     * @return The callback ID.
     */
    virtual int get_callback_id(const int pr) const = 0;

    /**
     * Returns the callback type for the given node.
     * @param pr The node identifier.
     * @return The callback type.
     */
    virtual int get_callback_type(const int pr) const = 0;

    /**
     * Returns the mutex ID for the given node.
     * @param pr The node identifier.
     * @return The mutex ID.
     */
    virtual int get_mutex_id(const int pr) const = 0;

    /**
     * Returns the runtime for the given node.
     * @param pr The node identifier.
     * @return The runtime.
     */
    virtual int get_runtime(const int pr) const = 0;
  };

} // namespace DFS

#endif
