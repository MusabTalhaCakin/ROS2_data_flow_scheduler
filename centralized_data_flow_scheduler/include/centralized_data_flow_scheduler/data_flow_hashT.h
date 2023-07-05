/*******************************************************************************
 * Copyright (C) 2023 TTTech Auto AG. All rights reserved                      *
 * Operngasse 17-21, 1040 Vienna, Austria. office(at)tttech-auto.com           *
 ******************************************************************************/

#ifndef DFS_HASHT_H
#define DFS_HASHT_H

#include "centralized_data_flow_scheduler/data_flow_types.h"
#include "centralized_data_flow_scheduler/data_flow_hashT.h"
#include "centralized_data_flow_scheduler/igraphcreator.h"

namespace DFS
{

  class DFSHashT
  {
  public:
    /**
     * Constructor for DFSHashT.
     */
    DFSHashT(){};

    /**
     * Destructor for DFSHashT.
     */
    ~DFSHashT()
    {
      std::cout << "[+]Destructor DFSHashT!\n";
    };

    /**
     * Creates the inarc and outarc tables based on the graph.
     * @param node_name The name of the node.
     * @param graph The graph used for creating the tables.
     */
    void create_tables(const std::string &, lemon::ListDigraph &);

    /**
     * Prints the inarc table.
     */
    void print_inarc_table() const;

    /**
     * Prints the outarc table.
     */
    void print_outarc_table() const;

    /**
     * Resets the inarc tables.
     */
    void reset_table();

    /**
     * Erases the executed tasks from the inarc tables.
     * @param executed_tasks Vector of the executed callbacks of the last iteration.
     */
    void erase_executed_tasks(const std::vector<int> &);

    /**
     * Returns the reference to the inarc table.
     * @return Reference to the inarc table.
     */
    std::map<int, std::vector<int>> &return_inarc()
    {
      return inarc_d;
    }

    /**
     * Returns the reference to the outarc table.
     * @return Reference to the outarc table.
     */
    std::map<int, std::vector<int>> &return_outarc()
    {
      return outarc_st;
    }

  private:
    std::string node_name;
    std::map<int, std::vector<int>> inarc_st;  // Static Inarc table
    std::map<int, std::vector<int>> inarc_d;   // Inarc table
    std::map<int, std::vector<int>> outarc_st; // Outarc table
  };
} // PAS

#endif