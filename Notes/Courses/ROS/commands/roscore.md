You **must** have a roscore running in order for ROS nodes to communicate.
<mark style="background: #FF5582A6;">Has to run in a separate terminal!</mark>

- NOTE: If you use `roslaunch`, it will automatically start roscore if it detects that it is not already running

roscore will start up:
- a ROS [[Master]]
- a ROS [[Parameter Server]]
- a [[rosout]] logging node

You can also specify a port to run the master on:
	`roscore -p 1234`