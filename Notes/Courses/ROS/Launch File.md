
### Example of a .launch file:
```xml
<launch>
	<!-- initialize a existing or new parameter -->
	<param name="/number_publis_frequency" type="double" value="3.0" />

	<!-- Start a python node with a launch file -->
	<node name="number_publisher" pkg="my_robot_tutorial" 
		type="number_publisher.py" />
	<!-- Start a cpp executable node with a launch file -->
	<node name="number_publisher" pkg="my_robot_tutorial" 
		type="number_counter" />
</launch>
```



### Create reusable launch files
- Can be done using the `<arg>` tag, which creates a variable with a possible default value:
```xml
<launch>
	<arg name="debug" default="fales" />
	...
	<group if="$(arg debug)">
		<param name="print_debug" value"true"/>
	</group>
	...
<\launch>
```
- which can be used of affect the behavior of the launch file and can be set when launching: `roslaunch launch_file.launch debug:=true`

- You can also include another launch file from another package which can also use the defined arguments:
```xml
<launch>
	...
	<include file="$(find package_name)/launch/file.launch">
		<arg name="test" value="$(debug)"/>
	</include>
	...
<\launch>

```