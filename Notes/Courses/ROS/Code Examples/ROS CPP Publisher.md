## Simplest Publisher
```cpp
#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char **argv){
	ros::init(argc, argv, "robot_news_radio_transmitter");
	ros::NodeHandle nh;
	
	ros::Publisher pub = nh.advertise<std_msgs::String>
		("/robot_news_radio", 10);
	
	ros::Rate rate(3);
	while(ros::ok()){
		std_msgs::String msg;
		msg.data = "Hi this is the radio";
		pub.publish(msg);
		rate.sleep();
	}
}
```
- <mark style="background: #FF5582A6;">Changes to the</mark> [[CMakeLists.txt]]:
```Cmake
add_executable(first_cpp_node src/my_first_node.cpp)
add_executable(robot_news_cpp_node src/robot_news_radio_transmitter.cpp)

target_link_libraries(first_cpp_node
	${catkin_LIBRARIES}
)
target_link_libraries(robot_news_cpp_node
	${catkin_LIBRARIES}
)
```
