Used to enable [[Nodes]] to locate each other and allows them therefore to communicate.

- Provides the [[Parameter Server]]
- Is run using [[roscore]] command

## Example:
For instance, let's say we have two Nodes; a <mark style="background: #FFB86CA6;">Camera node</mark> and an <mark style="background: #FFB86CA6;">Image_viewer node</mark>. A typical sequence of events would start with <mark style="background: #FFB86CA6;">Camera notifying the master that it wants to publish images on the topic "images"</mark>:
![ROS_master_example_english_1.png](https://wiki.ros.org/Master?action=AttachFile&do=get&target=ROS_master_example_english_1.png "ROS_master_example_english_1.png") 

Now, Camera publishes images to the "images" topic, but <mark style="background: #FFB86CA6;">nobody is subscribing to that topic yet</mark> so no data is actually sent. Now, Image_viewer wants to subscribe to the topic "images" to see if there's maybe some images there:
![ROS_master_example_english_2.png](https://wiki.ros.org/Master?action=AttachFile&do=get&target=ROS_master_example_english_2.png "ROS_master_example_english_2.png") 

Now that the topic "images" has both a publisher and a subscriber, <mark style="background: #FFB86CA6;">the master node notifies Camera and Image_viewer about each others existence so that they can start transferring images</mark> to one another:
![ROS_master_example_english_3.png](https://wiki.ros.org/Master?action=AttachFile&do=get&target=ROS_master_example_english_3.png "ROS_master_example_english_3.png")