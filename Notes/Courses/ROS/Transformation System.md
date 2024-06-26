- Keeps track of all coordinate frames over time, and allows you to ask questions like: ^8e7b22
	- Where was the head frame relative to the world frame, 5 seconds ago?
	- What is the pose of the object in my gripper relative to my base?
	- What is the current pose of the base frame in the map frame?![[Pasted image 20240204120546.png]]
- Implemented as a publisher/subscriber model on <mark style="background: #FFB86CA6;">the topics /tf and /tf_static</mark>
	- /tf_static is used for coordinate frames that don't change with respect to each other. For example a camera_frame and a camera_mount_frame
- Lets the user transform points, vectors, etc. between coordinate frames.

## Command line
- `rosrun tf tf_monitor`: prints information about the current transform tree
- `rosrun tf tf_echo source_frame target_frame`: prints information about the transformation between 2 frames
- `rosrun tf2_tools view_frames.py` to get an pdf overview of the tree structure:
	- ![[Screenshot from 2024-02-04 12-21-28.png]]

## RVIZ
- TF can also be visualized using `rviz`
- ![[Screenshot from 2024-02-04 12-23-13.png]]

## C++ API
- Create a TF listener to fill up a buffer:
```cpp
tf2_ros::Buffer tfBuffer;
tf2_ros::TransformListener tfListener(tfBuffer);
```
- To look up transformations:
```cpp
geometry_msg::TransformStamped transformStamped = tfBuffer.lookupTransform(target_frame_id, source_frame_id, time)
```
- For time, use `ros::Time(0)` for the latest available transform