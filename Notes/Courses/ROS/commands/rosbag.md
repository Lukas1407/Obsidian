Can record and playback data.

`rosbag record -a` records data with the option -a, indicating that all published topics should be accumulated in a bag file.
`rosbag record -O subset /turtle1/cmd_vel /turtle1/pose` the -O argument tells `rosbag record` to log to a file named subset.bag, and the topic arguments cause `rosbag record` to only subscribe to these two topics.

`rosbag info <your bagfile>` checks the contents of the bag file without playing it back.

`rosbag play <your bagfile>` replay the bag file to reproduce behavior in the running system.