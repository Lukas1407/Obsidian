
- calls the service if requested
## Easiest Service Client
```python
#!/usr/bin/python3

import rospy
from rospy_tutorials.srv import AddTwoInts 
from std_msgs.msg import Int64

import random

class AddTwoIntsClient():
    def __init__(self) -> None:
        self.add_two_ints_service = rospy.ServiceProxy("/add_two_ints", AddTwoInts)
    
    def call_add_two_ints(self, a:Int64, b:Int64):
        self.add_two_ints_service(a, b)

if __name__ == "__main__":
    rospy.init_node("add_two_ints_client")

    rospy.wait_for_service("/add_two_ints") # blocks until service is ready

    rate = rospy.Rate(1)
    try:
        add_two_ints_client = AddTwoIntsClient()
        while not rospy.is_shutdown():
            a = random.randint(0, 100)
            b = random.randint(-100, 100)
            add_two_ints_client.call_add_two_ints(a, b)
            rate.sleep()
    except rospy.ServiceException as e:
        rospy.logwarn("Service failed: " + str(e))
```
- Here it is sill called at 1 Hz -> real services would make this depended on some condition like time, data availability,...