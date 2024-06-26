- Shared dictionary that [[Nodes]] can access to store and retrieve parameters at runtime
- It is best used for <mark style="background: #FFB86CA6;">static, non-binary data</mark> such as configuration parameters.
- <mark style="background: #FFB86CA6;">Can store integers, floats, boolean, dictionaries, and lists</mark>
- The corresponding command is [[rosparam]]

Examples for Parameters:
```r
/camera/left/name: "leftcamera"
/camera/left/exposure: 1
/camera/right/name: "rightcamera"
/camera/right/exposure: 1.1
/simulation_mode: false
```
- The parameter /camera/left/name has the value leftcamera. You can also get the value for /camera/left, which is the dictionary:
```R
name: leftcamera
exposure: 1
```
- And you can also get the value for /camera, which has a dictionary of dictionaries representation of the parameter tree:
```r
left: { name: leftcamera, exposure: 1 }
right: { name: rightcamera, exposure: 1.1 }
```

### config.yaml
- The parameters can also be stored inside a config file stored inside the [[Packages|my_package/config folder]]. 
- Example:
```yaml
camera:
	left:
		name: leftcamera
		exposure: 1
	right:
		name: rightcamera
		exposure: 1.1
```
- Which then can be used inside a [[roslaunch#Example of a .launch file|Launch file]]: 
```xml
<launch>
	<node name="name" pkg="package" type="node_type">
		<rosparam command="load" file="$(find package)/config/config.yaml" />
	</node>
</launch>
```

### Interacting with parameters from Code
1. <mark style="background: #FFB86CA6;">Get a parameter</mark>: you can optionally set a default value if the parameter is not set
```python
default_param = rospy.get_param('/default_param')
default_param = rospy.get_param('/default_param', default_value) # if the parameter doesnt exist
```
```cpp
std::string s;
node_handle.get_param('default_param', s)
int i;
node_handle.param("my_num", i, 42); // allows to define a default value
```
2. <mark style="background: #FFB86CA6;">Set/Create a parameter</mark>:
```python
rospy.set_param('/a_string', 'baz')
```
```cpp
node_handle.setParam("/my_param", "hello there");
```
3. <mark style="background: #FFB86CA6;">Test parameter existence</mark>:
```python
if rospy.has_param('/to_delete'):
```
```cpp
node_handle.hasParam("my_param")
```
4. <mark style="background: #FFB86CA6;">Deleting a parameter</mark>:
```python
try:
    rospy.delete_param('/to_delete')
except KeyError:
    print("value not set")
```
```cpp
node_handle.deleteParam("/my_param");
```