Usage:
```
rosservice list         print information about active services
rosservice call         call the service with the provided args
rosservice type         print service type
rosservice find         find services by service type
rosservice uri          print service ROSRPC uri
```

## Examples
1. `rosservice list` for turtlesim
	- shows us that the turtlesim node provides nine services:
	```
	/clear
    /kill
    /reset
    /rosout/get_loggers
    /rosout/set_logger_level
    /spawn
    /teleop_turtle/get_loggers
    /teleop_turtle/set_logger_level
    /turtle1/set_pen
    /turtle1/teleport_absolute
    /turtle1/teleport_relative
    /turtlesim/get_loggers
    /turtlesim/set_logger_level
	```
2. `rosservice type` for turtlesim
	- displays the service type of a given service. The service type is the package name plus the name of the .srv file that defines the request and response messages for the service. For example, the service type of /clear is std_srvs/Empty, which means it is defined in the std_srvs package and the Empty.srv file.
	- <mark style="background: #FFB86CA6;">Arguments can be seen using</mark> `rosservice type /spawn | rossrv show`:
		```C
		float32 x
		float32 y
		float32 theta
		string name
		```
string name

1. `rosservice call` for turtlesim
	- calls the service with the necessary arguments
	- `rosservice call /spawn 3 0 0.2 "test"`