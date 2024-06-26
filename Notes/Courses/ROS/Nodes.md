- A Node is a process that performs computation
- Nodes are combined into a graph and communicate with one another using [[Topics]], [[Services]], and [[Parameter Server]]
- <mark style="background: #FFB86CA6;">Are meant to operate on a fine-grained scale</mark>
	-> a robot controller usually consists of many nodes. For example, one node controls a laser range-finder, one Node controls the robot's wheel motors, one node performs localization, one node performs path planning, one node provides a graphical view of the system, and so on.

- Similar idea to using classes in OOP

## Node Examples
- [[ROS Python Node|Example of a Node in python]]
- [[ROS CPP Node|Example of a Node in cpp]]
- 

## Node names
- All running nodes have a unique [[Graph Resource Name]]
	- For example, /hokuyo_node could be the name of a Hokuyo driver broadcasting laser scans.
- Nodes also have a _node type_, that simplifies the process of referring to a node executable on the fileystem. These node types are [[package resource names]] with the name of the node's package and the name of the node executable file.


## Remapping Arguments
ROS allows to change the name of a Node at runtime which allows <mark style="background: #FFB86CA6;">reusing the same node for different purposes</mark> from the command-line.
- `name:=new_name` is the syntax for remapping a name to a new name. For example, `chatter:=/wg/chatter` means that the node will use **/wg/chatter** instead of **chatter** as the topic name.
- `rosrun turtlesim turtlesim_node __name:=my_turtle` makes the name of the node now called "my_turtle" instead of "turtlesim"
The names are <mark style="background: #ADCCFFA6;">matched exactly</mark>, which means that only the whole name can be remapped, not parts of it. For example, **foo:=bar** will not match **foo/baz**, but only **foo** or **/<node_namespace>/foo**

Examples:

| Node Namespace | **Remapping Argument** | Match Names for the remapping in the Namespace | **Final Resolved Name** |
| -------------- | ---------------------- | ---------------------------------------------- | ----------------------- |
| /              | foo:=bar                    | foo, /foo                                               | /bar                        |
| <mark style="background: #ADCCFFA6;">/baz</mark>           | foo:=bar                    | foo, <mark style="background: #ADCCFFA6;">/baz/foo</mark>                                               | <mark style="background: #ADCCFFA6;">/baz/bar</mark>                        |
| /     | /foo:=bar                    | foo, /foo                                               | /bar                        |
| /baz           | /foo:=bar                     | /foo                                               | /baz/bar                        |
| /baz           | /foo:=/a/b/c/bar                     | /foo                                               | /a/b/c/bar                        |

## Node Parameter assignment
A parameter of a node can be assigned from the command-line by using a single underscore: `rosrun rospy_tutorials talker _param:=1.0`
	-> sets ~param to 1.0

 