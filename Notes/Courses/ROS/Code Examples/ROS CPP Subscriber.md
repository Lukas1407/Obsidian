## Easiest Subscriber
```cpp
#include <ros/ros.h>
#include <std_msgs/String.h>

void callback_recieve_radio(const std_msgs::String& msg){
	ROS_INFO("Message revieved: ");
	ROS_INFO(msg.data.c_str());
}

int main(int argc, char **argv){
	ros::init(argc, argv, "smartphone");
	ros::NodeHandle nh;
	
	ros::Subscriber sub = nh.subscribe("/robot_news_radio", 10, 
		callback_recieve_radio);
	
	ros::spin();
}
```
- <mark style="background: #FF5582A6;">Changes to the</mark> [[CMakeLists.txt]]:
```Cmake
add_executable(first_cpp_node src/my_first_node.cpp)
add_executable(robot_news_cpp_node src/robot_news_radio_transmitter.cpp)
add_executable(smartphone_cpp_node src/smartphone.cpp)

target_link_libraries(first_cpp_node
	${catkin_LIBRARIES}
)
target_link_libraries(robot_news_cpp_node
	${catkin_LIBRARIES}
)
target_link_libraries(smartphone_cpp_node
	${catkin_LIBRARIES}
)
