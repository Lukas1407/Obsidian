## Easiest Subscriber
```python
#!/usr/bin/python3
import rospy
from std_msgs.msg import String

def callback_recieve_radio_data(msg):
	rospy.loginfo("Message recieved: ")
	rospy.loginfo(msg)

if __name__ == "__main__":
	rospy.init_node("smartphone")
	
	sub = rospy.Subscriber("/robot_news_radio", String, callback_recieve_radio_data)
	rospy.spin() # keeps the node running
```
- <mark style="background: #FFB86CA6;">create a Node named "smartphone" that is subscribed to the topic "/robot_news_radio"</mark>

## Object Oriented Subscriber
```python
#!/usr/bin/python3

import rospy
from std_msgs.msg import Int64

class NumberCounter():
    def __init__(self) -> None:
        self.sub = rospy.Subscriber("/number", Int64, 
	        self.callback_counter, queue_size=10)

    def callback_counter(self, msg:Int64):
        self.counter += msg.data
        rospy.loginfo(self.counter)

if __name__ == "__main__":
    rospy.init_node("number_counter")
    number_counter = NumberCounter()
    rospy.spin()
```