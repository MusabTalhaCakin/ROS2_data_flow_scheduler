# 1 to N subscription
There are 3 nodes:

- `Node1`: produces `topic1` based on ROS2 timer timeout
- `Node2`: subscribed to `topic1` and produces `topic2`
- `Node3`: subscribed to `topic1` and `topic2`. Callback is only fired when both topics are available.



```
digraph G {
  Node1 -> Node1  [label="Timer"]
  Node1 -> Node2  [label="topic1"];
  Node1 -> Node3  [label="topic1"];
  Node2 -> Node3  [label="topic2"];
}
```


Check the [DAG](https://dreampuf.github.io/GraphvizOnline/#digraph%20G%20%7B%0A%20%20Node1%20-%3E%20Node1%20%20%5Blabel%3D%22Timer%22%5D%0A%20%20Node1%20-%3E%20Node2%20%20%5Blabel%3D%22topic1%22%5D%3B%0A%20%20Node1%20-%3E%20Node3%20%20%5Blabel%3D%22topic1%22%5D%3B%0A%20%20Node2%20-%3E%20Node3%20%20%5Blabel%3D%22topic2%22%5D%3B%0A%7D).

## Run
Start the sequencer indicating how many nodes are going to be scheduled: **3**
```
ros2 run data_flow_scheduler data_flow_scheduler 3
```
Start Node1, publishes a topic based on a timer expiration.

0 is node id, 1000000 us timeout, Node's name: Node1, Topic: topic1
```
ros2 run data_flow_scheduler template_node 0 1000000 Node1 topic1
```
Start Node2 that is subscribed to topic1 and publishes topic2
```
ros2 run data_flow_scheduler template_node 1 1000000 Node2 topic2 topic1
```
Start Node3 that is subscribed to topic1 and topic2 and publishes topic3 and topic4
```
ros2 run data_flow_scheduler multisub 1 1000000
```

Timeout supervision of callbacks can be changed through the `DFSched::TimeSupervision` enumeration and recompiling the nodes.