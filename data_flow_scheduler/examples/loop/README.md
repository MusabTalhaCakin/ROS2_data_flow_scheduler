# Chain and 1 to N subscription
There are 8 nodes:

- `Node1`: produces `topic1` based on ROS2 timer timeout
- `Node2`: subscribed to `topic1` and produces `topic2`
- `Node3`: subscribed to `topic1` and produces `topic3` 
- `Node4`: subscribed to `topic2` and produces `topic4`
- `Node5`: subscribed to `topic2` and produces `topic5`
- `Node6`: subscribed to `topic2` and produces `topic6`
- `Node7`: subscribed to `topic2` and produces `topic7`
- `Node7`: subscribed to `topic1`, `topic4`, `topic5`, `topic6` and `topic7` produces `topic8`



```
digraph G {
  Node1 -> Node1  [label="Timer"]
  Node1 -> Node2  [label="topic1"];
  Node1 -> Node3  [label="topic1"];
  Node1 -> Node8  [label="topic1"];
  
  Node2 -> Node4  [label="topic2"];
  Node2 -> Node5  [label="topic2"];  
  
  Node3 -> Node6  [label="topic3"];
  Node3 -> Node7  [label="topic3"];  
  
  Node4 -> Node8  [label="topic4"];
  Node5 -> Node8  [label="topic5"];
  Node6 -> Node8  [label="topic6"];
  Node7 -> Node8  [label="topic7"];
}
```


Check the [DAG](https://dreampuf.github.io/GraphvizOnline/#digraph%20G%20%7B%0A%20%20Node1%20-%3E%20Node1%20%20%5Blabel%3D%22Timer%22%5D%0A%20%20Node1%20-%3E%20Node2%20%20%5Blabel%3D%22topic1%22%5D%3B%0A%20%20Node1%20-%3E%20Node3%20%20%5Blabel%3D%22topic1%22%5D%3B%0A%20%20Node1%20-%3E%20Node8%20%20%5Blabel%3D%22topic1%22%5D%3B%0A%20%20%0A%20%20Node2%20-%3E%20Node4%20%20%5Blabel%3D%22topic2%22%5D%3B%0A%20%20Node2%20-%3E%20Node5%20%20%5Blabel%3D%22topic2%22%5D%3B%20%20%0A%20%20%0A%20%20Node3%20-%3E%20Node6%20%20%5Blabel%3D%22topic3%22%5D%3B%0A%20%20Node3%20-%3E%20Node7%20%20%5Blabel%3D%22topic3%22%5D%3B%20%20%0A%20%20%0A%20%20Node4%20-%3E%20Node8%20%20%5Blabel%3D%22topic4%22%5D%3B%0A%20%20Node5%20-%3E%20Node8%20%20%5Blabel%3D%22topic5%22%5D%3B%0A%20%20Node6%20-%3E%20Node8%20%20%5Blabel%3D%22topic6%22%5D%3B%0A%20%20Node7%20-%3E%20Node8%20%20%5Blabel%3D%22topic7%22%5D%3B%0A%7D%0A).

## Run
Start the sequencer indicating how many nodes are going to be scheduled: **8**
```
ros2 run data_flow_scheduler data_flow_scheduler 8
```
Start Node1, publishes a topic based on a timer expiration.

0 is node id, 300000 us timeout, Node's name: Node1, Topic: topic1
```
ros2 run data_flow_scheduler loop 0 300000 Node1 topic1
```
Start the rest of the nodes
```
ros2 run data_flow_scheduler loop 1 300000 Node2 topic1 topic2 
ros2 run data_flow_scheduler loop 1 300000 Node3 topic1 topic3 

ros2 run data_flow_scheduler loop 1 300000 Node4 topic2 topic4 
ros2 run data_flow_scheduler loop 1 300000 Node5 topic2 topic5 
ros2 run data_flow_scheduler loop 1 300000 Node6 topic3 topic6 
ros2 run data_flow_scheduler loop 1 300000 Node7 topic3 topic7 
```


## Timeout supervision

There are 3 diferent types of timeout supervision:

- By amount of time
- By amount of Thread time
- No supervision
  

This can be changed through the `DFSched::TimeSupervision` enumeration and recompiling the nodes.

### `DFSched::TimeSupervision::Time`
The system monotonic time is used to check if a callback run out of time. In case of timeout, the thread that is executing the callback is detached and it is let to execute.

A fail in the execution is reported to the user.

### `DFSched::TimeSupervision::ThreadCPUTime`
Instead of using the monotonic time, the thread CPU time is used.

### `DFSched::TimeSupervision::None`
There is no timeout supervision. A callback never timeouts.