- Topics are named busses over which [[Nodes]] exchange [[Messages]].
	-> unidirectional, streaming communication

- <mark style="background: #FFB86CA6;">Nodes that are interested in data subscribe to the relevant topic;
nodes that generate data publish to the relevant topic</mark>.

 - <mark style="background: #FFB86CA6;">Multiple Nodes can publish and subscribe to the same topic</mark>

- <mark style="background: #FF5582A6;">Nodes that want to receive a response to a request should use</mark> [[Services]]!

- The default transport mechanism is TCP/ID known as [[TCPROS]].

- [[rostopic]] is used to get information about the topic

## Example
run `rosrun turtlesim turtlesim_node` and `rosrun turtlesim turtle_teleop_key`
-> both nodes communicate with each other, turtle_teleop_key is **publishing** the key strokes on a topic, while turtlesim **subscribes** to the same topic to receive the key strokes.

- <mark style="background: #FFB86CA6;">To get a dynamic graph to see the topics and nodes use</mark> [[rqt_graph]]: `rosrun rqt_graph rqt_graph` ![[Pasted image 20240126132619.png]]
- <mark style="background: #FFB86CA6;">To get a dynamic plot of the data being published use</mark> [[rqt_plot]]: `rosrun rqt_plot rqt_plot`![[Pasted image 20240126133800.png]]