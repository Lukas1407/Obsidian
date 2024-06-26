- Provides the logic of the [[Services|service]]
## Easiest Service Server
```python 
#!/usr/bin/python3

import rospy
from rospy_tutorials.srv import AddTwoInts 

class AddTwoIntsServer():
    def __init__(self) -> None:
        self.service = rospy.Service("/add_two_ints", AddTwoInts, self.handle_add_two_ints)
    
    def handle_add_two_ints(self, req:AddTwoInts):
        sum = req.a + req.b
        rospy.loginfo(f"Sum of {req.a} + {req.b} = {sum}")
        return sum

if __name__ == "__main__":
    rospy.init_node("add_two_ints_server")
    add_two_ints_server = AddTwoIntsServer()
    
    rospy.spin()
```