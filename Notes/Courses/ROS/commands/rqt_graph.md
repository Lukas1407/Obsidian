plots the interaction between [[Nodes]], like [[Topics]]
- called with `rqt_graph`

## Example
- When running <mark style="background: #FFB86CA6;">2 nodes named "my_first_python_node" and "my_first_cpp_node"</mark> <mark style="background: #FF5582A6;">which are not communicating</mark>!![[Pasted image 20240207140731.png]]
- When running <mark style="background: #FFB86CA6;">2 nodes named "turtlesim" and "teleop_turtle"</mark>. Teleop_turtle sends the <mark style="background: #FFB86CA6;">message "/turtle1/cmd_vel"</mark> to "turtlesim". "turtle1" is the namespace for the topic.![[Pasted image 20240207141043.png]]