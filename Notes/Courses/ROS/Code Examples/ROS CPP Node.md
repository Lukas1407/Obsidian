- Is stored in the "src" folder

## Easiest cpp Node
```cpp
#include <ros/ros.h>
int main (int argc, char **argv) {
	ros::init(argc, argv, "my_first_cpp_node");
	ros::NodeHandle nh; // needed to start the node in cpp
	
	ROS_INFO("Node has been started");
	ros::Duration(1.0).sleep(); 
	ROS_INFO("Exit the Node");
}
```
- <mark style="background: #FF5582A6;">Needed changes</mark> to the [[CMakeLists.txt]]:
```cmake
# Declare a C++ executable
# With catkin_make all packages are built within a single CMake context
# The recommended prefix ensures that target names across packages don't collide
add_executable(my_cpp_node src/my_first_node.cpp)
# Specify libraries to link a library or executable target against
target_link_libraries(my_cpp_node
	${catkin_LIBRARIES}
)
```

## Sleep
```cpp
#include <ros/ros.h>
int main (int argc, char **argv) {
	ros::init(argc, argv, "my_first_cpp_node");
	ros::NodeHandle nh; // needed to start the node in cpp
	ROS_INFO("Node has been started");
	
	ros::Rate rate = ros::Rate(10);
	while (ros::ok()){ // while the node is running
		ROS_INFO("hello");
		rate.sleep(); // to ensure the computation is at 10hz
	}
}
```

## Anonymous Nodes
```python
...
ros::init(argc, argv, "my_first_cpp_node", 
		  ros::init_options::AnonymousName);
...
```
- Allows to start multiple nodes with the same name, as it adds random numbers to it's name:![[Pasted image 20240207153759.png]]
