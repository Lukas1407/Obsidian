
## Node Example
```python
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int64

class NumberCounterNode(Node):
	def __init__(self):
		super().__init__("number_counter")
		self.counter = 0
		self.number_counter_publisher = self.create_publisher(Int64, "numer_counter", 10)
		self.number_subscriber = self.create_substription(Int64, "number", self.callback_number, 10)

	def callback_number(self, msg):
		self.counter += msg.data
		self.get_logger().info("Counter: " + str(self.counter))

def main(args=None):
	rclpy.init(args=args)
	node = NumberCounterNode()
	rclpy.spin(node)
	rclpy.shutdown()
```