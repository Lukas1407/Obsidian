displays information about the [[Messages]]

## show
Display the fields in a ROS message type.
Example: `rosmsg show sensor_msgs/CameraInfo` returns:
```c
Header header
  uint32 seq
  time stamp
  string frame_id
uint32 height
uint32 width
RegionOfInterest roi
  uint32 x_offset
  uint32 y_offset
  uint32 height
  uint32 width
float64[5] D
float64[9] K
float64[9] R
float64[12] P
```

## list
Display a list of all messages.
Example: `rosmsg list` returns:
```c
nav_msgs/GridCells
nav_msgs/MapMetaData
nav_msgs/OccupancyGrid
nav_msgs/Odometry
nav_msgs/Path
...
```