## Easiest Service Client
```cpp
#include <ros/ros.h>
#include <rospy_tutorials/AddTwoInts.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "add_two_ints_client");
    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<rospy_tutorials::AddTwoInts>("/add_two_ints");

    rospy_tutorials::AddTwoInts srv;
    srv.request.a = 10;
    srv.request.b = 4;

    if (client.call(srv)) {
        // call successfull -> we can process the data
        int sum = (int)srv.response.sum;
        ROS_INFO("Returned sum: %d", sum);
    } else {
        ROS_WARN("Service call failed");
    }
}   
```
- <mark style="background: #FF5582A6;">Add it to the CmakeLists</mark>