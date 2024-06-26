- First coined by Karel Capek (1920) with "robata" (slovenian) = hard work

## 4 Generations of Robotics
1. Programmable Manipulators (1960 - 1975)
	- Low computational abilities
	- Only point-to-point programmable
	- Low sensory abilities (pick-and-place)
2. Adaptive Robots (1976 - 1982)
	- More sensors
	- Adaptation to the environment
	- Low intelligence
3. Autonomous Robots (1983 - )
	- High computational abilities
	- Task-oriented programming
	- Due to need of autonomy
4. [[Humanoid Robots]]
	- Flexible adaption to environment and tasks
	- Able to learn
	- Emotions
## Service Robots
- Aid the human in daily tasks
- Universal control for different tasks
- Sensing of dynamic environment
- Interaction and communication ability
- Able to learn and adapt
- Solving complex tasks on its own

## Signal Processing
### Optical
- Signals are transmitted through the use of light, maybe of different colors
- Need direct sight or through optical fiber/glass fiber
- Advantages:
	- Robust against electromagnetic disturbances
	- Low weight of optical fibers
	- High transfer rate, because of no loss over distance
	- Bandwidth of 60 THz
- Disadvantages:
	- Difficult to place
	- Limited bend potential
	- Because of missing storage and processing elements, conversion to electrical signal often necessary
### Electrical
- Signals are transmitted through electrical impulses through copper cables
- Two-wire or coaxial cable
- Advantages:
	- Easy to place
	- Flexible cable
- Disadvantages:
	- Prone to electromagnetic disturbances
	- Low bandwidth
### Radio
- Signals are transmitted through electromagnetic waves through air
- Basis for [[Bluetooth]], [[WiFi]], 2G and 3G
- Advantages:
	- No placing of cables
- Disadvantages:
	- Low stability and data security
	- Low bandwidth
	- Prone to interference frequencies

## Materials of Robots
Robots consist of multiple materials. The materials are chosen based on their [[Material Science#Characteristics of Materials|characteristics]] and the function they need to do!
For example:
- Steel or aluminium for complex load parts
- Fiber composite materials for directional load
- 3D prints for extreme light weight but low load parts

## Kinematics/Morphology
### Joint types
1. Linear Joints:
	- Linear joints facilitate parallel motion between adjacent links. In other words, the input and output links slide in a straight line, resulting in translational motion. These joints are essential for tasks like pick-and-place, where precise linear movement is required.
2. Rotary Joints:
	- Rotary joints allow rotational motion between the links. There are three types of rotary joints:
	1. Revolute Joints (Hinge Joints): These joints enable rotation around a single axis. Think of a door hinge—it allows the door to swing open and closed. Revolute joints are common in robotic arms for tasks like assembly and welding.
	2. Prismatic Joints (Linear Joints): Unlike revolute joints, prismatic joints allow linear movement along a single axis. Imagine a drawer sliding in and out—it’s a prismatic joint. These are useful for extending or retracting parts of a robot.
	3. Fixed Joints: These joints do not allow any movement. They keep adjacent links rigidly connected. While fixed joints are not directly involved in motion, they provide stability to the overall robot structure.
3. Other Types:
	- Twisting Joints: These joints allow twisting or torsional motion between links. They are less common but find applications in specialized robots.
	- Orthogonal Joints: These joints create motion at right angles to each other. They combine linear and rotational movements.
	- Revolving Joints: These joints allow circular motion around an axis. Think of a rotating platform or turntable.
![[Pasted image 20240229104944.png#invert|]]
- Problem with ball-and-socket joint:
	- Are very complex to model, as they allow movement along 3 axis
	- Its usually approximated using 3 rotational joints
	- ![[Pasted image 20240229105139.jpg|200]]

## Drive Types
1. Pneumatic: Expanding and compressing air
	- Very fast movements possible
	- Need air compressor
2. Hydraulic: Using some fluid like water
	- A lot of force possible
3. Electric: 
	- Smooth motions possible
![[Pasted image 20240301085125.png#invert|600]]


## Controlling the motion of Robots
1. [[Position Control]]
2. [[Velocity Control]]
3. [[Torque Control]]
In practice, most robots use a combination of these control methods, depending on the situation and the goal. 
For example, a robot may switch from position control to torque control when it makes contact with an object, or from velocity control to position control when it reaches a desired location. 
Some advanced control techniques, such as impedance control or admittance control, can also adjust the stiffness or damping of the robot to achieve a desired behavior.
- [[PID Controller]]

## Sensors in Robotics
[[Sensor|Sensors]] in robots are based on the functions of human [[Sensory Organs]]. Robots require extensive information about their environment in order to function effectively.
There are 2 different types of [[Sensor|sensors]] used in robotics:
### Internal Sensors
- Give information about the internal state of the robot
1. Position Sensors
2. Velocity Sensors
3. Acceleration Sensors
4. Navigation Sensors
### External Sensors
- Give information about the environment
1. Proximity Sensors
2. Touch Sensors
3. Acceleration Sensors
4. Visual Sensors
5. Position Sensors