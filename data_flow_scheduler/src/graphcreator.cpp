/*******************************************************************************
 * Copyright (C) 2023 TTTech Auto AG. All rights reserved                      *
 * Operngasse 17-21, 1040 Vienna, Austria. office(at)tttech-auto.com           *
 ******************************************************************************/

#include "data_flow_scheduler/graphcreator.h"

using namespace DFS;

GraphCreator::GraphCreator()
{
  // Initialize node maps
  node_id = std::make_unique<lemon::ListDigraph::NodeMap<int>>(graph_);
  sub = std::make_unique<lemon::ListDigraph::NodeMap<std::string>>(graph_);
  pub = std::make_unique<lemon::ListDigraph::NodeMap<std::string>>(graph_);
  runtime = std::make_unique<lemon::ListDigraph::NodeMap<int>>(graph_);
  longest_path = std::make_unique<lemon::ListDigraph::NodeMap<int>>(graph_);
  callback_id = std::make_unique<lemon::ListDigraph::NodeMap<int>>(graph_);
  callback_type = std::make_unique<lemon::ListDigraph::NodeMap<int>>(graph_);
  mutex_id = std::make_unique<lemon::ListDigraph::NodeMap<int>>(graph_);
}

void GraphCreator::build_graph(const DFS_Interface::NodeInfoVector &nodeinfo_vec)
{
  // Iterate over each node in the nodeinfo_vec
  for (const auto &node : nodeinfo_vec)
  {
    int j = 0;
    // Add nodes to the graph and set node properties
    for (const auto &callback : node.callback_topic_name)
    {
      lemon::ListDigraph::Node node_ = graph_.addNode();
      (*node_id)[node_] = node.node_id;
      (*sub)[node_] = callback;
      (*pub)[node_] = node.pub_topic_name[j];
      (*runtime)[node_] = node.runtime[j];
      (*callback_id)[node_] = node.callback_id[j];
      (*callback_type)[node_] = node.callback_type[j];
      (*mutex_id)[node_] = j;
      j++;
    }
  }

  // Connect nodes with edges based on topic matching
  for (lemon::ListDigraph::NodeIt n(graph_); n != lemon::INVALID; ++n)
  {
    for (lemon::ListDigraph::NodeIt m(graph_); m != lemon::INVALID; ++m)
    {
      if (n != m)
      {
        if ((*pub)[m] == (*sub)[n])
          graph_.addArc(m, n);
      }
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("GraphCreator"), "Graph created (DIA Graph with %d nodes and %d arcs).", lemon::countNodes(graph_), lemon::countArcs(graph_));
  compute_longestpath();
}

void GraphCreator::compute_longestpath()
{
  run_time = std::make_unique<lemon::ListDigraph::ArcMap<int>>(graph_);
  lemon::ListDigraph::Node end = graph_.addNode();
  // Assign negative runtime values to the arcs and connect end node
  for (lemon::ListDigraph::NodeIt n(graph_); n != lemon::INVALID; ++n)
  {
    if (lemon::countOutArcs(graph_, n) == 0 && n != end)
      graph_.addArc(n, end);
    for (lemon::ListDigraph::OutArcIt a(graph_, n); a != lemon::INVALID; ++a)
    {
      (*run_time)[a] = -1 * (*runtime)[n];
    }
  }
  // Reverse arcs for Bellman-Ford algorithm
  for (int i = 0; i < lemon::countArcs(graph_); i++)
  {
    graph_.reverseArc(graph_.arcFromId(i));
  }

  // Run Bellman-Ford algorithm to compute longest path
  lemon::BellmanFord<lemon::ListDigraph, lemon::ListDigraph::ArcMap<int>> bf(graph_, *run_time);
  bf.run(end);

  // Store longest path distances in the longest_path node map
  for (lemon::ListDigraph::NodeIt node_it(graph_); node_it != lemon::INVALID; ++node_it)
  {
    if (node_it != end)
      (*longest_path)[node_it] = -1 * bf.dist(node_it);
  }

  // Reverse arcs back to their original direction
  for (int i = 0; i < lemon::countArcs(graph_); i++)
  {
    graph_.reverseArc(graph_.arcFromId(i));
  }
  // Remove the end node from the graph
  graph_.erase(end);
}

void GraphCreator::save_to_file(const std::string &path) const
{
  // Save the graph to a file in LGF format
  lemon::DigraphWriter<lemon::ListDigraph> writer(graph_, path);
  writer.nodeMap("SOCKET", *node_id);
  writer.nodeMap("ID", *callback_id);
  writer.nodeMap("TYPE", *callback_type);
  writer.nodeMap("RUNTIME", *runtime);
  writer.nodeMap("LONGEST_PATH", *longest_path);
  writer.nodeMap("NAME", *sub);
  writer.run();

  RCLCPP_INFO(rclcpp::get_logger("GraphCreator"), "LGF Graph interface created.");
}