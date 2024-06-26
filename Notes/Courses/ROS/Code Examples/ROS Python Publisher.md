## Easiest Publisher
```python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

if __name__ == "__main__":
	rospy.init_node("robot_news_radio_transmitter")
	# topic_name, topic_type, queue_size: he size of the buffer where the messages are saved in
	pub = rospy.Publisher("/robot_news_radio", String, queue_size=10)
	  
	rate = rospy.Rate(2)
	while not rospy.is_shutdown():
		msg = String()
		msg.data = "Hi to the robot news"
		pub.publish(msg)
		rate.sleep()
```
- publishes "Hi to the robot news" at 2Hz to the topic named "/robot_news_radio"![[Pasted image 20240207145244.png]]

## Object Oriented Publisher
```python
#!/usr/bin/python3
import rospy
from std_msgs.msg import Int64
import random

class NumberPublisher():
	def __init__(self) -> None:
		self.pub = rospy.Publisher("/number", Int64, queue_size=10)
		
	def publish_data(self):
		msg = Int64()
		msg.data = random.randint(0, 10)
		self.pub.publish(msg)

if __name__ == "__main__":
	rospy.init_node("number_publisher")
	number_publisher = NumberPublisher()
	
	rate = rospy.Rate(2)
	while not rospy.is_shutdown():
		number_publisher.publish_data()
		rate.sleep()
```