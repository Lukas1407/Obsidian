- In python the Nodes are typically located in a "scripts" folder
- do `chmod +x file_name` after creation to make it executable
	-> Makes them able to run with`rosrun

## Easiest python Node
```python
#!/usr/bin/env python3
import rospy

if __name__ == "__main__":
	rospy.init_node("my_first_python_node") # specifiy unique name
	rospy.loginfo("This node has been started")
	rospy.sleep(1) # amount of seconds
	rospy.loginfo("Exit now")
```
- Consists just of a main function with `rospy.init_node("name")`
- Starting a node with the same name will kill the first node

## Sleep
```python
#!/usr/bin/env python3
import rospy

if __name__ == "__main__":
	rospy.init_node("my_first_python_node") # specifiy unique name
	rospy.loginfo("This node has been started")

rate = rospy.Rate(10) # in herz
while not rospy.is_shutdown(): # is true if node is killed
	rospy.loginfo("hello")
	rate.sleep() # keepts the rate of execution at 10 herz
```

## Anonymous Nodes
```python
...
rospy.init_node("my_first_python_node", anonymous=True)
...
```
- Allows to start multiple nodes with the same name, as it adds random numbers to it's name:![[Pasted image 20240207153759.png]]
