- create a new package just for launch files called `xxx_bringup`
- delete all folders and add folder called `launch`
- In package.xml add the dependencies to the package where the nodes are we want to launch
- in cmakeList add:
```
install(DIRECTORY
	launch
	DESTINATION share/${PROJECT_NAME}$
)
```

## Example
- Convention: call the launch file `test.launch.py`
- make it executable
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	ld = LaunchDescription()

	remap_number_topic = ("number", "my_number")

	number_publisher_node = Node(
		package="ros2_test",
		executable="number_publisher",
		name="number_publisher",
		remappings=[
			remap_number_topic],
		parameters=[
			{"number_to_publish": 4}]
	)

	ld.add_action(number_publisher_node)
	return ld
```
