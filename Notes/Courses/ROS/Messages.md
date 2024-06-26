[[Nodes]] communicate with each other by publishing [[Messages]] to [[Topics]].

- A Message is a simple data structure with integers, floats, boolean, arrays etc.
- Nodes can also exchange a _request_ and _response_ message as part of a ROS [[Services]] call. These request and response messages are defined in srv files.

Message names: <mark style="background: #FFB86CA6;">the name of the package + / + name of the .msg file</mark>
- For example, `std_msgs/msg/String.msg` has the message type `std_msgs/String`.

## Define [[Messages]], [[Services]] and [[Actions]] in a folder:
- <mark style="background: #FF5582A6;">This should be in a separate folder</mark> from the [[Packages]]!
- -> Other packages must only include the messages folder, not the entire package
```
- my_package_msgs/
      actions
      msg
      srv
      CMakeLists.txt
      package.xml
```

## msg file
- Text file that specifies the data structure of the message
- Stored in the msg sub directory of the package `msg/`
- consists of data field descriptions and constant definitions
```c
int32 X=123
int32 Y=-123
int32 y
string s // you **cannot** leave a comment on a string constant definition.
float32[] ranges // variable length array
float32[10] points // fixed size array
```

- The <mark style="background: #FFB86CA6;">field name determines how a data value is referenced in the target language</mark>. For example, a field called 'pan' would be referenced as 'obj.pan' in Python, assuming that 'obj' is the variable storing the message.
- Field names must be an alphabetical character followed by any mixture of alphanumeric and underscores
#### Built-in types:

| **Primitive Type** | **Serialization** | **C++** | **Python2**/**Python3** |
| ---- | ---- | ---- | ---- |
| bool | unsigned 8-bit int | uint8_t | bool |
| int8 | signed 8-bit int | int8_t | int |
| uint8 | unsigned 8-bit int | uint8_t | int (3) |
| int16 | signed 16-bit int | int16_t | int |
| uint16 | unsigned 16-bit int | uint16_t | int |
| int32 | signed 32-bit int | int32_t | int |
| uint32 | unsigned 32-bit int | uint32_t | int |
| int64 | signed 64-bit int | int64_t | long/int |
| uint64 | unsigned 64-bit int | uint64_t | long/int |
| float32 | 32-bit IEEE float | float | float |
| float64 | 64-bit IEEE float | double | float |
| string | ascii string (4) | std::string | str/bytes |
| time | secs/nsecs unsigned 32-bit ints | [ros::Time](http://docs.ros.org/api/rostime/html/classros_1_1Time.html) | [rospy.Time](http://www.ros.org/doc/api/rospy/html/rospy.rostime.Time-class.html) |
| duration | secs/nsecs signed 32-bit ints | [ros::Duration](http://docs.ros.org/api/rostime/html/classros_1_1Duration.html) | [rospy.Duration](http://www.ros.org/doc/api/rospy/html/rospy.rostime.Duration-class.html) |


#### Header.msg
Header type provides a special mechanism to <mark style="background: #ADCCFFA6;">setting frame IDS</mark>
- <mark style="background: #ADCCFFA6;">A frame ID is a way of identifying the coordinate system of a point</mark>, vector, or frame. Frame IDs are useful for transforming data between different reference frames, such as a robot’s base frame and a camera’s frame.

Header is not a built-in type (it's <mark style="background: #FFB86CA6;">defined in std_msgs/msg/Header.msg</mark>) and it contains the following fields:
- **uint32 seq**: A sequence number that increments with each message sent. This is useful for detecting message drops or reordering.
- **time stamp**: The time at which the message was generated. This is useful for synchronizing data from different sources or performing time-based calculations.
- **string frame_id**: The name of the frame this data is associated with. This is usually a fixed frame, such as “base_link” or “map”, but it can also be a moving frame, such as “odom” or “camera”.
```c
#Standard metadata for higher-level flow data types
#sequence ID: consecutively increasing ID
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
```

## Necessary changes to the XML and CMakeLists
#### Simple version:
- Add the line rosbuild_genmsg() to your [[CMakeLists.txt]] file:
```cmake
cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rosbuild_init()
rosbuild_genmsg()
```

#### Advanced version:
- Open [[package.xml]] and make sure these two lines are in it:
 ```xml
  <build_depend>message_generation</build_depend>
  <run_depend>message_runtime</run_depend>
```
- Open [[CMakeLists.txt]] and add the message_generation dependency:
```cmake
# Do not just add this line to your CMakeLists.txt, modify the existing line
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation)
```
- Also make sure you export the message runtime dependency.
```cmake
catkin_package(
  ...
  CATKIN_DEPENDS message_runtime ...
  ...)
```
- Find the following block of code:
```cmake
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )
```
- Uncomment it by removing the # symbols and then replace Message*.msg files with your .msg file, such that it looks like this:
```cmake
add_message_files(
  FILES
  Num.msg
)
```
- Find the following block of code:
```cmake
# generate_messages(
#    DEPENDENCIES
#    std_msgs  # Or other packages containing msgs
# )
```
- Uncomment it by removing the # symbols and then replace std_msgs with the messages your messages depend on, such that it looks like this:
```cmake
 generate_messages(
   DEPENDENCIES
   std_msgs
)
```
