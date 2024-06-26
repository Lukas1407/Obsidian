Allows you to store and manipulate data on the ROS [[Parameter Server]].

Usage:
```
rosparam set [param_name]           set parameter
rosparam get [param_name]           get parameter
rosparam load           load parameters from file (usually .yaml)
rosparam dump           dump parameters to file (usually .yaml)
rosparam delete         delete parameter
rosparam list           list parameter names
```

## Examples
1. `rpsparam list` for turtlesim
	- Turtlesim node has three parameters on the param server for background color:
```
	/rosdistro
    /roslaunch/uris/host_nxt__43407
    /rosversion
    /run_id
    /turtlesim/background_b
    /turtlesim/background_g
    /turtlesim/background_r
```
2. `rosparam set` for turtlesim
	- `rosparam set /turtlesim/background_r 150` changes the parameter value, so now <mark style="background: #FF5582A6;">we have to call the clear service for the parameter change to take effect</mark>: `rosservice call /clear`![[Pasted image 20240126141016.png]]
3. `rosparam get /` to get the entire content of the parameter server