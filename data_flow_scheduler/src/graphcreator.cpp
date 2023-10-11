/*******************************************************************************
 * Copyright (C) 2023 TTTech Auto AG. All rights reserved                      *
 * Operngasse 17-21, 1040 Vienna, Austria. office(at)tttech-auto.com           *
 ******************************************************************************/

#include "data_flow_scheduler/graphcreator.h"
#include <lemon/core.h>

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
  supervision = std::make_unique<lemon::ListDigraph::NodeMap<int>>(graph_);
}

/*void GraphCreator::build_graph(const DFSched::NodeInfoVector
&nodeinfo_vec)
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

  RCLCPP_INFO(rclcpp::get_logger("GraphCreator"), "Graph created (DIA Graph with
%d nodes and %d arcs).", lemon::countNodes(graph_), lemon::countArcs(graph_));
  compute_longestpath();
}*/

lemon::ListDigraph::NodeIt GraphCreator::find_node_by_id(int id) const
{
  for (lemon::ListDigraph::NodeIt node_(graph_); node_ != lemon::INVALID;
       ++node_)
  {
    if ((*node_id)[node_] == id)
    {
      return node_;
    }
  }

  return lemon::INVALID;
}

lemon::ListDigraph::NodeIt
GraphCreator::find_node_by_published_topic(const std::string &topic) const
{
  for (lemon::ListDigraph::NodeIt node_(graph_); node_ != lemon::INVALID;
       ++node_)
  {
    if ((*pub)[node_] == topic)
    {
      return node_;
    }
  }

  return lemon::INVALID;
}

void GraphCreator::build_graph(const DFSched::NodeInfoVector &nodeinfo_vec)
{

  std::map<std::string, lemon::ListDigraph::NodeIt> map_topicpublished_node;

  // Create all the nodes
  for (const auto &node : nodeinfo_vec)
  {
    lemon::ListDigraph::Node node_ = graph_.addNode();
    (*node_id)[node_] = node.node_id;
  }

  // Iterate over each node in the nodeinfo_vec

  for (const auto &node : nodeinfo_vec)
  {
    int j = 0;

    auto node_ = find_node_by_id(node.node_id);
    // find the node

    if (node_ == lemon::INVALID)
    {
      continue;
    }

    (*node_id)[node_] = node.node_id;
    if (node.callback_topic_name.empty())
    {
      (*sub)[node_] = nullptr;
    }
    else
    {
      for (auto sub_topic : node.callback_topic_name)
      {
        (*sub)[node_] = sub_topic + "," + (*sub)[node_];
      }
    }

    if (node.pub_topic_name.empty())
    {
      (*pub)[node_] = nullptr;
    }
    else
    {

      for (auto pub_topic : node.pub_topic_name)
      {
        (*pub)[node_] = pub_topic + "," + (*pub)[node_];

        map_topicpublished_node[pub_topic] = node_;
      }
    }

    (*runtime)[node_] = node.runtime[0];
    (*callback_id)[node_] = node.callback_id[0];
    (*callback_type)[node_] = node.callback_type[0];
    (*mutex_id)[node_] = j;
    (*supervision)[node_] = static_cast<int32_t>(node.supervision_kind);
  }

  // Connect nodes with edges based on topic matching
  for (const auto &node : nodeinfo_vec)
  {
    auto node_ = find_node_by_id(node.node_id);
    std::cout << "Checking node " << node.node_id << "\n";
    if (node_ != lemon::INVALID)
    {
      for (auto &sub : node.callback_topic_name)
      {

        auto pub_id = map_topicpublished_node.find(sub);
        if (pub_id != map_topicpublished_node.end())
        {
          if ((*node_id)[pub_id->second] != node.node_id)
          {
            std::cout << " |-> Subscription " << sub
                      << " comes from publisher id "
                      << (*node_id)[pub_id->second] << "\n";
            graph_.addArc(pub_id->second, node_);
          }
        }
        else
        {
          std::cout << " |-> ERROR! Subscription " << sub
                    << " is not published!\n";
        }
      }
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("GraphCreator"),
              "Graph created (DIA Graph with %d nodes and %d arcs).",
              lemon::countNodes(graph_), lemon::countArcs(graph_));
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
  lemon::BellmanFord<lemon::ListDigraph, lemon::ListDigraph::ArcMap<int>> bf(
      graph_, *run_time);
  bf.run(end);

  // Store longest path distances in the longest_path node map
  for (lemon::ListDigraph::NodeIt node_it(graph_); node_it != lemon::INVALID;
       ++node_it)
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
  writer.nodeMap("SUPERVISION", *supervision);
  writer.run();

  RCLCPP_INFO(rclcpp::get_logger("GraphCreator"),
              "LGF Graph interface created.");
}
