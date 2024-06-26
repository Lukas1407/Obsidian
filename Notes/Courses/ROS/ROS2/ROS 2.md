
## Build
- uses colcon
- `colcon build -symlink-install`
	- symlink does that we dont need to build every time we change the code
## Packages
- Are reusable independet code ensembles
- Created via: `ros2 pkg create test_package --build-type ament_python --dependencies rclpy`

## Node Template
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class TestNode(Node):
	def __init__(self, name):
		super().__init__(name)
		self.get_logger().info("Hello ROS2")
		
def main():
	rclpy.init()
	node = TestNode("test_node")
	rclpy.spin(node)
	rclpy.shutdown()

if __name__=="__main__":
	main()
```
- do `chmod +x <file_name>` to make it executable
- `ros2 run packkage pub_node`
- in setup.py do:
```python
entry_points={
	'console_scripts': [
		"pub_node = test_pkg.test_node:main",
		"sub_node = test_pkg.subscriber_node:main"
],
```
### Publisher
```python
class TestNode(Node):
	def __init__(self, name):
		super().__init__(name)
		self.test_pub = self.create_publisher(String, "test_node/test_topic", 10)  
		
		self.create_timer(1, self.publish_test) 
	
	def publish_test(self):
		msg = String()
		msg.data = "test"
		self.test_pub.publish(msg)
```
### Subscriber
```python
from example_interfaces.msg import String
class TestNode(Node):
	def __init__(self, name):
		super().__init__(name)
		self.test_sub = self.create_subscription(String, "test_node/test_topic", qos_profile=10, callback=self.test_callback)
	
	def test_callback(self, msg):
		self.get_logger().info(f"got message: {msg.data}")
```
### Service Server
```python 
class TestNode(Node):
	def __init__(self, name):
		super().__init__(name)
		self.server = self.create_service(AddTwoInts, "add_two_ints", self.callback_add_two_ints)  
	
	def callback_add_two_ints(self, request, response):
		response.sum = request.a + request.b
		return response
```
### Service Client
```python 
class TestNode(Node):
	def __init__(self, name):
		super().__init__(name)
		self.call_add_two_ints_server(6, 8)
	
	def call_add_two_ints_server(self, a, b):
		client = self.create_client(AddTwoInts, "add_two_ints")
		client.wait_for_service()
		request = AddTwoInts.Request()
		request.a = a
		request.b = b
		
		future = client.call_async(request)
		future.add_done_callback(self.done_callback)
	
	def done_callback(self, future):
		try:
			resopnse = future.result()
			self.get_logger().info(f"call complete: sum = {resopnse.sum}")
		except Exception as e:
			self.get_logger().error("Service call failed")
```

## Custom messages
- Create a new Package for the messages with folders `msg srv action` delete all other folders
- in package.xml add:
```
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```
- in cmakeList add:
```
find_package(rosidl_default_generators REQUIRED)
ament_export_dependencies(rosidl_default_runtime)
```
### Usage
Add to the package.xml in the package where we want to use the new message: `<depend>"name of message package"</depend>`

## Changes to ROS1
- Instead of [[rospy]] is now [[rclpy]], same for c++.
- Introduces a convention on how to program a node:
	- define a class which inherits `Node`
- Introduces [[ROS2 Components|Components]]
- Launch files now use python: [[Launch Files ROS2]]