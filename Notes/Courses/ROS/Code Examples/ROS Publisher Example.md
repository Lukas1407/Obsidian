[[Create A ROS Package]]

1. Create a scripts folder to store the Python scripts:
```
mkdir scripts
cd scripts
```

2. Then download the example script [talker.py](https://raw.github.com/ros/ros_tutorials/kinetic-devel/rospy_tutorials/001_talker_listener/talker.py) to your new scripts directory and make it executable:
```
wget https://raw.github.com/ros/ros_tutorials/kinetic-devel/rospy_tutorials/001_talker_listener/talker.py
chmod +x talker.py
```

3. Download the [listener.py](https://raw.github.com/ros/ros_tutorials/kinetic-devel/rospy_tutorials/001_talker_listener/listener.py) file into your scripts directory:
```
roscd beginner_tutorials/scripts/
wget https://raw.github.com/ros/ros_tutorials/kinetic-devel/rospy_tutorials/001_talker_listener/listener.py
chmod +x listener.py
```

4. Add the following to your CMakeLists.txt. This makes sure the python script gets installed properly, and uses the right python interpreter.
```cmake
catkin_install_python(PROGRAMS scripts/talker.py scripts/listener.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```
5. Build the Nodes:
```
cd ~/catkin_ws
catkin_make
```
### talker.py explanation
```python
#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
```


``` python
pub = rospy.Publisher('chatter', String, queue_size=10)
```
This section of code defines the talker's interface to the rest of ROS. pub = rospy.Publisher("chatter", String, queue_size=10) <mark style="background: #FFB86CA6;">declares that your node is publishing to the chatter topic using the message type String</mark>. String here is actually the class std_msgs.msg.String. The queue_size argument limits the amount of queued messages if any subscriber is not receiving them fast enough.
```python
rospy.init_node('talker', anonymous=True)
```
Tells rospy the name of your node -- until rospy has this information, it cannot start communicating with the [[Master]]. In this case, your node will take on the name talker. NOTE: the name must be a [base name](https://wiki.ros.org/Names), i.e. it cannot contain any slashes "/".
`anonymous = True` ensures that your node has a unique name by adding random numbers to the end of NAME.
```python
rate = rospy.Rate(10) # 10hz
```
This line creates a Rate object rate. With the help of its method sleep(), it offers a convenient way for looping at the desired rate.
```python
while not rospy.is_shutdown():
    hello_str = "hello world %s" % rospy.get_time()
    rospy.loginfo(hello_str)
    pub.publish(hello_str)
    rate.sleep()
```
This loop is a fairly standard rospy construct: checking the `rospy.is_shutdown()` flag and then doing work. You have to check `is_shutdown()` to check if your program should exit (e.g. if there is a Ctrl-C or otherwise). In this case, the "work" is a call to `pub.publish(hello_str)` that publishes a string to our chatter topic. The loop calls `rate.sleep()`, which sleeps just long enough to maintain the desired rate through the loop.
This loop also calls `rospy.loginfo(str)`, which performs triple-duty: the messages get printed to screen, it gets written to the Node's log file, and it gets written to [[rosout]].

### listener.py explanation
```python
#!/usr/bin/env python
import rospy
from std_msgs.msg import String
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("chatter", String, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
if __name__ == '__main__':
    listener()
```

```python
rospy.init_node('listener', anonymous=True)
rospy.Subscriber("chatter", String, callback)
# spin() simply keeps python from exiting until this node is stopped
rospy.spin()
```
This declares that your node subscribes to the chatter topic which is of type std_msgs.msgs.String. When new messages are received, callback is invoked with the message as the first argument. `rospy.spin()` simply keeps your node from exiting until the node has been shutdown.