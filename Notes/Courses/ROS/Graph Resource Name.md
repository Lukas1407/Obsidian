- Used to identify all resources in the ROS Computational Graph such as [[Nodes]], [[Topics]], [[Parameter Server]], and [[Services]].
- Each resource is defined within a namespace, which it may share with many other resources.
- <mark style="background: #FFB86CA6;">Resources can _create_ resources within their namespace and they can _access_ resources within or above their own namespace.</mark>
		-> Similar to class hierarchy 
- Connections can be made between resources in distinct namespaces, but this is generally done by integration code above both namespaces.

## Legal Names
A valid name has the following characteristics:
1. First character is an alpha character (a-z|A-Z), tilde (~) or forward slash (/)
2. Subsequent characters can be alphanumeric (0-9|a-z|A-Z), underscores (\_), or forward slashes (\/)
Examples:
- /
- /foo
- /stanford/robot/name
- <mark style="background: #ADCCFFA6;">/wg/node1</mark>

node <mark style="background: #ADCCFFA6;">/wg/node1</mark> has the namespace /wg

Names that start with a "~" are _private_.