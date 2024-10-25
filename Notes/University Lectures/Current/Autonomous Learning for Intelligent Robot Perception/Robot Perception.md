## Robot Perception Pipeline
### 1. Data Acquisition and Sensor Calibration
- The robot takes measurements from the environment via its sensors
### 2. Environment Modeling
- Using this data, a model of the environment ist calculated
- E.g. a point cloud, colors, temperature,...
### 3. Semantic Interpretation
- The meaning behind the pixels/voxels from the environment model is calculated
### 4. Location and Navigation
- Finally the robot has everything needed to interact with the world
## Challenges in Robot Perception
- Mapping of large-scale outdoor environments
- Object and environment classification
- Object pose estimation and grasp planing
- 3D simulation and reconstruction
![[Pasted image 20241025121537.png#invert|700]]
## Perception for Manipulation
- 3 main perception tasks for manipulation
### Object detection
- **Segmentation + Feature Extraction**: The robot identifies objects by segmenting the scene and extracting features (like shape or edges).
- **Adds Semantic Information**: This step provides "meaning" to objects—helping the robot differentiate, for example, a cup from a book.
- **Classification Problem**: Object detection involves sorting objects into categories (e.g., identifying that a specific item is a cup).
- **Appearance Requirements**: The robot uses characteristics like color and texture to identify objects, so these features are essential in this step.
### Object pose estimation
- **Position and Orientation**: This task determines where an object is in space (position) and how it is angled (orientation), so the robot knows exactly where it is relative to the camera.
- **Regression Problem**: Unlike classification, this is about predicting continuous values (e.g., exact coordinates and angles) rather than categories.
- **Geometry Requirement**: The robot needs detailed information about the object’s shape to precisely estimate its pose.
### Grasp detection
- **Gripper Pose**: It calculates the best way to position the gripper to hold the object securely.
- **Regression Problem**: Again, this is about predicting exact values (the angle and position of the gripper).
- **Geometry and Kinematics**: The robot must understand both the shape of the object and the motion of the gripper (like the way a hand moves) to create a stable grasp.
### Common Pipeline
![[Pasted image 20241025121919.png#invert|700]]
- Grasp detection is not necessary if we already know the object model and therefore know how to grasp it
### 3 Main Problems
- **Inverse Problems**: These problems focus on finding a plausible cause for a given effect, which is challenging because multiple causes could result in the same effect.
    - **In Object Detection and Grasp Detection**: For object detection, the robot is given a visual scene (the effect) and has to determine what objects are in it (the cause). For grasp detection, it must find a proper gripper pose based on an object’s shape and location, even though many poses could potentially work. Both tasks require the robot to reverse-engineer the scene from limited visual clues, which can be complex and ambiguous.
- **Open-world Assumption**: This assumption means that the robot must always be prepared for unfamiliar objects to appear in its environment.
    - **Unknown Objects**: Robots must handle objects they haven’t seen before, requiring them to generalize beyond their training. This is difficult because distinguishing unknown objects from known ones and making decisions about them (like whether they’re safe to interact with) is inherently uncertain.
- **Real-time Constraints**: These tasks must be performed quickly, often in real-time, because robots usually operate in dynamic environments.
    - **Need for Speed**: Robots have to process information rapidly and respond instantly. For example, a delay in detecting an object or adjusting a grip could lead to errors, collisions, or dropped objects. Real-time constraints make it challenging to perform complex calculations within limited time frames.