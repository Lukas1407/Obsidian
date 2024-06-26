annotation-target:: pdfs/Exercise Session 1.pdf





>%%
>```annotation-json
>{"created":"2024-01-29T12:24:30.791Z","text":"rostopic list to get all possible topics\n\nrostopic type /cmd_vel to get the type of the topic we want to publish\n\nrosmsg type geometry_msg/Twist to get the type of message we want to publish\n\ndone by:\nrostopic pub /cmd_vel geometry/Twist -r 10 -- '[0.5, 0.0, 0.0]' '[0.0, 0.0, 0.3]'","updated":"2024-01-29T12:24:30.791Z","document":{"title":"Exercise Session 1.pdf","link":[{"href":"urn:x-pdf:16d353f923093318dc57108a597e6993"},{"href":"vault:/ROS/pdfs/Exercise Session 1.pdf"}],"documentFingerprint":"16d353f923093318dc57108a597e6993"},"uri":"vault:/ROS/pdfs/Exercise Session 1.pdf","target":[{"source":"vault:/ROS/pdfs/Exercise Session 1.pdf","selector":[{"type":"TextPositionSelector","start":802,"end":859},{"type":"TextQuoteSelector","exact":"Command a desired velocity to the robot from the terminal","prefix":"http://wiki.ros.org/rosnode  3. ","suffix":" (​rostopic pub [TOPIC]​) (Lectu"}]}]}
>```
>%%
>*%%PREFIX%%http://wiki.ros.org/rosnode  3.%%HIGHLIGHT%%Command a desired velocity to the robot from the terminal%%POSTFIX%%(​rostopic pub [TOPIC]​) (Lectu*
>%%LINK%%[[#^w0hp078xikp|show annotation]]
>%%COMMENT%%
>rostopic list to get all possible topics
>
>rostopic type /cmd_vel to get the type of the topic we want to publish
>
>rosmsg type geometry_msg/Twist to get the type of message we want to publish
>
>done by:
>rostopic pub /cmd_vel geometry/Twist -r 10 -- '[0.5, 0.0, 0.0]' '[0.0, 0.0, 0.3]'
>%%TAGS%%
>
^w0hp078xikp


>%%
>```annotation-json
>{"created":"2024-01-29T12:27:00.935Z","text":"run by using rosrun teleop_twist_keyboard teleop_twist_keyboard.py","updated":"2024-01-29T12:27:00.935Z","document":{"title":"Exercise Session 1.pdf","link":[{"href":"urn:x-pdf:16d353f923093318dc57108a597e6993"},{"href":"vault:/ROS/pdfs/Exercise Session 1.pdf"}],"documentFingerprint":"16d353f923093318dc57108a597e6993"},"uri":"vault:/ROS/pdfs/Exercise Session 1.pdf","target":[{"source":"vault:/ROS/pdfs/Exercise Session 1.pdf","selector":[{"type":"TextPositionSelector","start":916,"end":936},{"type":"TextQuoteSelector","exact":"eleop_twist_keyboard","prefix":" (Lecture 1 Slide 13)  4. Use ​t","suffix":" ​to control your robot using th"}]}]}
>```
>%%
>*%%PREFIX%%(Lecture 1 Slide 13)  4. Use ​t%%HIGHLIGHT%%eleop_twist_keyboard%%POSTFIX%%​to control your robot using th*
>%%LINK%%[[#^njx8fmgegs|show annotation]]
>%%COMMENT%%
>run by using rosrun teleop_twist_keyboard teleop_twist_keyboard.py
>%%TAGS%%
>
^njx8fmgegs


>%%
>```annotation-json
>{"created":"2024-01-29T13:24:29.335Z","text":"<?xml version=\"1.0\" encoding=\"utf-8\"?>\n\n<launch>\n\n  <include file=\"$(find smb_gazebo)/launch/smb_gazebo.launch\">\n    <arg name=\"world\" default=\"robocup14_spl_field\"/>\n  </include>\n\n</launch>\n","updated":"2024-01-29T13:24:29.335Z","document":{"title":"Exercise Session 1.pdf","link":[{"href":"urn:x-pdf:16d353f923093318dc57108a597e6993"},{"href":"vault:/ROS/pdfs/Exercise Session 1.pdf"}],"documentFingerprint":"16d353f923093318dc57108a597e6993"},"uri":"vault:/ROS/pdfs/Exercise Session 1.pdf","target":[{"source":"vault:/ROS/pdfs/Exercise Session 1.pdf","selector":[{"type":"TextPositionSelector","start":1216,"end":1227},{"type":"TextQuoteSelector","exact":"launch file","prefix":"git_cheat_sheet.pdf  5. Write a ","suffix":" with the following content (Lec"}]}]}
>```
>%%
>*%%PREFIX%%git_cheat_sheet.pdf  5. Write a%%HIGHLIGHT%%launch file%%POSTFIX%%with the following content (Lec*
>%%LINK%%[[#^yvn1irwtmum|show annotation]]
>%%COMMENT%%
><?xml version="1.0" encoding="utf-8"?>
>
><launch>
>
>  <include file="$(find smb_gazebo)/launch/smb_gazebo.launch">
>    <arg name="world" default="robocup14_spl_field"/>
>  </include>
>
></launch>
>
>%%TAGS%%
>
^yvn1irwtmum
