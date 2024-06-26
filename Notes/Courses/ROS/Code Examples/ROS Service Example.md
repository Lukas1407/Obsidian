[[Create A ROS Package]]
1. Create a scripts folder to store the Python scripts:
```
mkdir scripts
cd scripts
```
2. Add a "add_two_ints_server.py" file to it:
```python
#!/usr/bin/env python
from __future__ import print_function
from beginner_tutorials.srv import AddTwoInts,AddTwoIntsResponse
import rospy
def handle_add_two_ints(req):
    print("Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b)))
    return AddTwoIntsResponse(req.a + req.b)
def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    print("Ready to add two ints.")
    rospy.spin()
if __name__ == "__main__":
    add_two_ints_server()
```
3. Add a "add_two_ints_client.py" file to it:
```python
#!/usr/bin/env python
from __future__ import print_function
import sys
import rospy
from beginner_tutorials.srv import *
def add_two_ints_client(x, y):
    rospy.wait_for_service('add_two_ints')
    try:
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        resp1 = add_two_ints(x, y)
        return resp1.sum
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
def usage():
    return "%s [x y]"%sys.argv[0]
if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    else:
        print(usage())
        sys.exit(1)
    print("Requesting %s+%s"%(x, y))
    print("%s + %s = %s"%(x, y, add_two_ints_client(x, y)))
```
4. Make the node are executable: 
```
chmod +x scripts/add_two_ints_server.py
chmod +x scripts/add_two_ints_client.py
```
5. Add the following to your CMakeLists.txt. This makes sure the python script gets installed properly, and uses the right python interpreter.
```cmake
catkin_install_python(PROGRAMS 
	scripts/add_two_ints_server.py
	scripts/add_two_ints_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```
1. Build the nodes:
```
cd ~/catkin_ws
catkin_make
```

### add_two_ints_server.py explanation
```python
s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
```
Declares a new service named 'add_two_ints' of the type AddTwoInts. All requests are passed to the `handle_add_two_ints` function.

### add_two_ints_client.py explanation
```python
 rospy.wait_for_service('add_two_ints')
```
This is a convenience method that blocks until the service named 'add_two_ints' is available.
```python
add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
```
Create a function for handling the service, which can be used like this:
```python
resp1 = add_two_ints(x, y)
return resp1.sum
```