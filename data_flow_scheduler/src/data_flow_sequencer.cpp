/*******************************************************************************
 * Copyright (C) 2023 TTTech Auto AG. All rights reserved                      *
 * Operngasse 17-21, 1040 Vienna, Austria. office(at)tttech-auto.com           *
 ******************************************************************************/

#include "data_flow_scheduler/data_flow_sequencer.h"

using namespace DFS;
volatile sig_atomic_t DFSSequencer::signalReceived = 0;

void DFSSequencer::init(const std::string &name_)
{
  node_name = name_;
  // Initialize the hash tables and print the inarc and outarc tables
  lemon::ListDigraph &graph = gcreator.get_graph();
  hashtables.create_tables(node_name, graph);
  hashtables.print_inarc_table();
  hashtables.print_outarc_table();
}

void DFSSequencer::print_vector(const std::vector<int> &vec) const
{
  // Print the elements of a vector
  for (size_t i = 0; i < vec.size(); i++)
  {
    std::cout << " " << vec[i] << ", ";
  }
  std::cout << std::endl;
}

void DFSSequencer::print_runtime() const
{
  int min = -1, max = -1;
  for (size_t i = 0; i < runtime_count.size(); i++)
  {
    // Find the minimum and maximum runtime values
    if (i == 0)
    {
      min = runtime_count[i];
      max = runtime_count[i];
    }
    else
    {
      if (min > runtime_count[i])
        min = runtime_count[i];
      if (max < runtime_count[i])
        max = runtime_count[i];
    }
  }
  // Calculate the average runtime
  float avarage = std::accumulate(
                      runtime_count.begin(),
                      runtime_count.end(), 0.0) /
                  runtime_count.size();
  std::cout << "-----\n"
            << "Min. loop runtime: " << min << " μs\n"
            << "avarage loop runtime: " << avarage << " μs\n"
            << "Max. loop runtime: " << max << " μs\n"
            << "-----\n";
}

void DFSSequencer::all_dependet_nodes_executed(
    std::map<int, std::vector<int>> &inarc,
    std::map<int, std::vector<int>> outarc)
{
  bool already_in_list = false;
  for (size_t i = 0; i < executed_last.size(); i++)
  {
    for (size_t j = 0; j < outarc[executed_last[i]].size(); j++)
    {
      for (size_t k = 0; k < ready_list.size(); k++)
      {
        if (ready_list[k] == outarc[executed_last[i]][j])
          already_in_list = true;
      }
      if (already_in_list == false)
      {
        // Add nodes to the ready list if all incoming arcs are executed
        if (inarc[outarc[executed_last[i]][j]].empty())
        {

          ready_list.push_back(outarc[executed_last[i]][j]);
        }
      }
      already_in_list = false;
    }
  }
  executed_last.clear();
}

void DFSSequencer::execute_callback(int node_id,
                                    int graph_node_id,
                                    int runtime_,
                                    std::vector<int> &ready_list,
                                    int callback_id,
                                    int callback_type,
                                    int mutex_id,
                                    DFSServer &passerver)
{
  // Prepare the Execute_Info struct
  available_cores--;
  execute_.callb = callback_id;
  execute_.runtime = runtime_;
  execute_.pr = graph_node_id;
  execute_.type = callback_type;
  execute_.mtx_id = mutex_id;

  // Send the Execute_Info struct to the data flow scheduler
  passerver.send_raw_data(node_id, &execute_,
                          sizeof(DFS_Interface::Execute_Info));
  if (VERBOSE)
    RCLCPP_INFO(rclcpp::get_logger(node_name),
                "WRITE[%d] -> ID:%d | Type:%d | Runtime:%d",
                node_id, callback_id, callback_type, runtime_);

  // Remove the executed node from the ready list
  auto &vec = ready_list;
  auto it = std::find(vec.begin(), vec.end(), graph_node_id);
  if (it != vec.end())
    vec.erase(it);
}

void DFSSequencer::start_sequencer(DFSServer &passerver)
{
  RCLCPP_INFO(rclcpp::get_logger(node_name), "START the Sequencer!");
  setSignalHandler();
  lemon::ListDigraph &graph_ = gcreator.get_graph();
  while (!signalReceived)
  {
    first = true;
    ready_list.clear();
    executed.clear();
    executed_last.clear();
    auto start = std::chrono::high_resolution_clock::now();

    while (executed.size() != (unsigned int)countNodes(graph_))
    {
      if (VERBOSE)
      {
        std::cout << "-------\n";
        hashtables.print_inarc_table();
      }
      if (signalReceived)
      {
        break;
      }
      passerver.set_fd();
      if (first == true)
      {
        // Initialize the ready list with nodes that have no incoming arcs
        for (size_t i = 0; i < hashtables.return_inarc().size(); i++)
        {
          if (hashtables.return_inarc()[i].empty())
            ready_list.push_back(i);
        }
        first = false;
        if (VERBOSE)
        {
          std::cout << "[READY LIST] \n";
          print_vector(ready_list);
        }
      }
      else if (!executed_last.empty())
      {
        // Check if all dependent nodes are executed and add them to the ready list
        all_dependet_nodes_executed(hashtables.return_inarc(),
                                    hashtables.return_outarc());
        if (VERBOSE)
        {
          std::cout << "[READY LIST] \n";
          print_vector(ready_list);
        }
      }
      if (VERBOSE)
      {
        std::cout << "Wait for response ...\n";
      }
      for (int j = 0; j < CORES; j++)
      {
        if (signalReceived)
        {
          break;
        }
        if (!ready_list.empty() && available_cores > 0)
        {
          // Random
          /*
          std::srand(std::time(0));
          int randomIndex = std::rand() % ready_list.size();
          int pr = ready_list[randomIndex];
          */

          // longest path
          /**/
          int pr = ready_list[0];
          for (size_t k = 0; k < ready_list.size(); k++)
          {
            // Find the node with the longest path in the ready list
            if (gcreator.get_longestpath(pr) < gcreator.get_longestpath(ready_list[k]))
            {
              pr = ready_list[k];
            }
          }
          /**/
          // Execute the callback of the selected node
          execute_callback(
              gcreator.get_node_id(pr),
              pr,
              gcreator.get_runtime(pr),
              ready_list,
              gcreator.get_callback_id(pr),
              gcreator.get_callback_type(pr),
              gcreator.get_mutex_id(pr),
              passerver);
        }
      }
      if (!signalReceived)
      {
        // Handle responses from data flow scheduler
        // and update the executed and executed_last vectors
        bool ret = passerver.handle_response(
            execute_, available_cores, executed, executed_last);
        if (!ret)
          signalReceived = 1;
        hashtables.erase_executed_tasks(executed_last);
      }
      if (VERBOSE)
      {
        std::cout << "[EXECUTED LAST] \n";
        print_vector(executed_last);
        std::cout << "[EXECUTED TOTAL] \n";
        print_vector(executed);
        std::cout << "-------\n";
      }
    }

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    if (VERBOSE)
    {
      std::cout << " loop runtime: " << duration.count() << " μs\n";
    }
    runtime_count.push_back(duration.count());
    hashtables.reset_table();
  }
  print_runtime();
}