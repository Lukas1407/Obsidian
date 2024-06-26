- *[[Packages]]* contains libraries, scripts, or other artifacts
- *Manifests/[[package.xml]]* defines the dependencies between the packages and captures meta information about the packages like version, maintainer, licence,... 

## Command-line 
- [[rospack]] allows to get information about the package
- [[roscd]] allows to change directory directly to a package
- [[rosls]] lists files in a package
- [[rosnode]] displays information about a node
- [[rosmsg]] prints the message definition
- [[rostopic]] prints information about a topic
- [[roscore]] "starts" a ROS-based system. Must be the first thing to run
- [[rosout]]: ROS equivalent of stdout/stderr
- [[rosrun]] to start running a node inside a package
- [[rosservice]] used on services
- [[rosparam]] allows you to store and manipulate data on the ROS [[Parameter Server]]
- [[roslaunch]] starts Nodes as specified in a [[Launch File]]
- [[rosbag]] can record and play back data
- [[roswtf]] examines your system and tries to find problems
- [[roslaunch]] launches multiple [[Nodes]] defined in a .launch file

## Graph Concept of ROS
- [[Nodes]]: A node is an executable that uses ROS to communicate with other nodes.
- [[Messages]]: ROS data type used when subscribing or publishing to a topic.
- [[Topics]]: Nodes can _publish_ messages to a topic as well as _subscribe_ to a topic to receive messages.
- [[Services]] allow Nodes to send a request and receive a response 
- [[Master]]: Name service for ROS (i.e. helps nodes find each other)
- [[Parameter Server]]: Stores parameters for the nodes to access
- [[rosout]]: ROS equivalent of stdout/stderr
- [[roscore]]: Master + rosout + parameter server (parameter server will be introduced later)

## RQT
Used to visualize the ROS system, like [[Messages]], [[Topics]]
Its run using `rqt`
- [[rqt_graph]]
- [[rqt_plot]]
- [[rqt_console]]

## RViz
- 3D visualization tool for the ROS network
- visualizes the message contents 
- interactive tools to publish
- Run with `rviz`

## Transformation System TF
![[Transformation System#^8e7b22]]

## Unified Robot Description Format (URDF)
- XML format description of the robot 

## Simulation Description Format (SDF)
- XML format of environments, objects, sensors, robots
- URDF can be converted to SDF

## ROS 2
- [[ROS 2|Differences in ROS 2]]

## Example Implementations
1. [[ROS Python Node]]
2. [[ROS CPP Node]]
3. [[ROS Python Publisher]]
4. [[ROS CPP Publisher]]
5. [[ROS Python Subscriber]]
6. [[ROS CPP Subscriber]]
7. [[ROS Python Service Server]]
8. [[ROS Python Service Client]]
9. [[ROS CPP Service Server]]
10. [[ROS CPP Service Client]]
11. [[ROS Custom MSG]]

Less relevant official tutorials:
1. [[ROS Publisher Example]]
2. [[ROS Service Example]]
### ROS ETH Exercises
- [[ETH ROS Exercise 1]]
- [[ETH ROS Exercise 2]]
- [[ETH ROS Exercise 3]]