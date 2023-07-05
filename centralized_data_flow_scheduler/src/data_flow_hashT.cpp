/*******************************************************************************
 * Copyright (C) 2023 TTTech Auto AG. All rights reserved                      *
 * Operngasse 17-21, 1040 Vienna, Austria. office(at)tttech-auto.com           *
 ******************************************************************************/

#include "centralized_data_flow_scheduler/data_flow_hashT.h"

using namespace DFS;

void DFSHashT::create_tables(const std::string &name_, lemon::ListDigraph &graph)
{
  node_name = name_;
  for (lemon::ListDigraph::NodeIt n(graph); n != lemon::INVALID; ++n)
  {
    // Populate the incoming arcs for the current node
    if (!(countInArcs(graph, n) == 0))
    {
      for (lemon::ListDigraph::InArcIt a(graph, n); a != lemon::INVALID; ++a)
      {
        inarc_st[graph.id(n)].push_back(graph.id(graph.source(a)));
      }
    }
    else
    {
      // Add a placeholder value when there are no incoming arcs
      inarc_st[graph.id(n)].push_back(-1);
      inarc_st[graph.id(n)].erase(inarc_st[graph.id(n)].begin());
    }
    // Populate the outgoing arcs for the current node
    if (!(countOutArcs(graph, n) == 0))
    {
      for (lemon::ListDigraph::OutArcIt a(graph, n); a != lemon::INVALID; ++a)
      {
        outarc_st[graph.id(n)].push_back(graph.id(graph.target(a)));
      }
    }
    else
    {
      // Add a placeholder value when there are no outgoing arcs
      outarc_st[graph.id(n)].push_back(-1);
      outarc_st[graph.id(n)].erase(outarc_st[graph.id(n)].begin());
    }
  }
  inarc_d = inarc_st;
  RCLCPP_INFO(rclcpp::get_logger(node_name), "Hash Tables Created.");
}

void DFSHashT::reset_table()
{
  inarc_d = inarc_st;
}

// Iterate over the executed tasks from the last iteration
void DFSHashT::erase_executed_tasks(const std::vector<int> &executed_last)
{
  for (size_t i = 0; i < executed_last.size(); i++)
  {
    // Iterate over the outgoing arcs of the executed task
    for (size_t j = 0; j < outarc_st[executed_last[i]].size(); j++)
    {
      if (!outarc_st[executed_last[i]].empty())
      {
        auto &vec = inarc_d[outarc_st[executed_last[i]][j]];
        // Find and erase the executed task from the incoming arcs
        auto it = std::find(vec.begin(), vec.end(), executed_last[i]);
        if (it != vec.end())
        {
          vec.erase(it);
        }
      }
    }
  }
}

void DFSHashT::print_inarc_table() const
{
  std::cout << "[INARCS MAP]\n";
  std::map<int, std::vector<int>> arc = inarc_d;
  // Iterate through the map and print the elements
  std::map<int, std::vector<int>>::iterator it = arc.begin();
  while (it != arc.end())
  {
    std::cout << " ID: " << it->first << ", Values: ";
    for (size_t j = 0; j < it->second.size(); j++)
    {
      std::cout << it->second[j] << " ";
    }
    std::cout << std::endl;
    ++it;
  }
}

void DFSHashT::print_outarc_table() const
{
  std::cout << "[OUTARCS MAP]\n";
  std::map<int, std::vector<int>> arc = outarc_st;
  // Iterate through the map and print the elements
  std::map<int, std::vector<int>>::iterator it = arc.begin();
  while (it != arc.end())
  {

    std::cout << " ID: " << it->first << ", Values: ";
    for (size_t j = 0; j < it->second.size(); j++)
    {
      std::cout << it->second[j] << " ";
    }
    std::cout << std::endl;
    ++it;
  }
}