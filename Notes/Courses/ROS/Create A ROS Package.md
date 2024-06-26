```
# You should have created this in the Creating a Workspace Tutorial
cd ~/catkin_ws/src
```
Now use the catkin_create_pkg script to create a new package called 'beginner_tutorials' which depends on std_msgs, roscpp, and rospy:
`catkin_create_pkg beginner_tutorials std_msgs rospy roscpp`

Now you need to build the packages in the catkin workspace:
```
cd ~/catkin_ws
catkin_make
```

<mark style="background: #FF5582A6;">To add the workspace to your ROS environment you need to source the generated setup file:</mark> `. ~/catkin_ws/devel/setup.bash`
	-> has to be done in every new terminal
	- or run `echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc` 