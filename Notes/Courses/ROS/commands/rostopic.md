Prints information about [[Topics]], including publishers, subscribers, publishing rate and [[Messages]].

This is the current list of supported commands:
``` unix
rostopic bw     display bandwidth used by topic
rostopic delay  display delay for topic which has header
rostopic echo   print messages to screen
rostopic find   find topics by type
rostopic hz     display publishing rate of topic
rostopic info   print information about active topic
rostopic list [-v]  print information about active topics
rostopic pub    publish data to topic
rostopic type   print topic type
```

### Examples
1. `rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'`
	- `-1` causes rostopic to only publish one message then exit
	- `/turtle1/cmd_vel` is the name of the topic to publish to
	- `geometry_msgs/Twist` is the message type to use
	- `--` tells the option parser that none of the following arguments is an option. This is required in cases where your arguments have a leading dash -, like negative numbers.
	- `'[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'` the actual message