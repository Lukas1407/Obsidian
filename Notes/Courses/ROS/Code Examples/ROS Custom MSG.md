- Create a new package for the message -> good way to entangle complexity 
`catkin_create_pkg dependecies`
- create a `msg` folder and within that a `MessageName.msg`

## Example MessageName.msg
```
int64 temperature
bool are_motors_up
string debug_message
```

## Changes to the CMakeLists.txt
1. Add message_generation to `find_package`:
```Cmake
find_package(catkin REQUIRED COMPONENTS
  ...
  message_generation
)
```
2. Add the MessageName.msg to the message definition:
```Cmake
# Generate messages in the 'msg' folder
add_message_files(
  FILES
  HardwareStatus.msg
)
```
3. Add the dependencies of the new message:
```cmake
## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)
```

## Changes to the package.xml
```xml
  ...
  <build_depend>message_generation</build_depend>
  ...
  <exec_depend>message_runtime</exec_depend>
```

# Use the Message in a Node
1. Add the package name of message to the package.xml:
```xml
  ...
  <depend>my_robot_msgs</depend>
  ...
```
2. Add the package name of message to the CMakeLists.txt:
```Cmake
find_package(catkin REQUIRED COMPONENTS
  ...
  my_robot_msgs
)
```
3. Define a publisher for example that uses this message:
```python
#!/usr/bin/python3

import rospy
from my_robot_msgs.msg import HardwareStatus

class HardwareStatusPublisher():
    def __init__(self) -> None:
        self.pub = rospy.Publisher("/my_robot/hardware_status", HardwareStatus, queue_size=10)

    def publish(self):
        msg = HardwareStatus()
        msg.temperature = 45
        msg.are_motors_up = True
        msg.debug_message = "Everyting is running well"
        self.pub.publish(msg)

if __name__ == "__main__":
    rospy.init_node("hw_status_publisher")
    hardware_status_publisher = HardwareStatusPublisher()
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        hardware_status_publisher.publish()
        rate.sleep()
```