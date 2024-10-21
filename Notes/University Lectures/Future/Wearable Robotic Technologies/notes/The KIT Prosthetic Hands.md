## Papers
[[Weiner2018a.pdf]]
[[fnbot-16-815716.pdf]]

## Detail
The KIT Prosthetic Hands are advanced personalized prosthetic devices equipped with semi-autonomous grasping capabilities, specifically designed to mimic natural hand movements and sizes based on the 50th percentile male and female hand dimensions. These prosthetic hands are scalable to match the dimensions of a user’s able hand, enhancing both function and comfort. The construction utilizes durable plastics produced through selective laser sintering, a form of 3D printing.
### Key Features of the KIT Prosthetic Hands:
1. **Degrees of Freedom (DoF):**
   - 10 degrees of freedom: Includes individual motor control for the thumb and a single motor that actuates the flexion of all fingers.
2. **Underactuated Mechanism:**
   - The four fingers are actuated by one motor, connected through a system of tendons.
   - This setup utilizes an adaptive underactuated mechanism, which might include configurations like a Whippletree with pulley blocks or a double pulley block system. This allows the fingers to adjust naturally around the shape of the object they are grasping, providing flexibility and adaptability in grip.
3. **Embedded System:**
   - Features a powerful microcontroller (ARM, 400MHz) integrated directly into the hand.
   - This system processes sensor inputs, controls the motor functions, and facilitates wireless communication via Bluetooth Low Energy.
   - It also supports embedded vision and machine learning algorithms, enhancing the hand’s ability to interact intelligently with its environment.
4. **Multimodal Sensor System:**
   - Includes an Inertial Measurement Unit (IMU) to estimate the prosthesis' pose and can recognize user gestures to control the hand.
   - A distance sensor and an RGB camera provide spatial awareness and object recognition capabilities. The time of flight distance sensing is particularly useful for accurate operation in diverse lighting conditions.
5. **Design and Material:**
   - The prosthetic components are lightweight and robust, owing to the advanced materials and manufacturing methods employed.
These features collectively make the KIT Prosthetic Hands highly functional for users, mimicking natural hand movements while providing intelligent, adaptive responses to varied tasks and environments.

### Underactuated Mechanism
![[Pasted image 20240715125736.png#invert|600]]
![[Pasted image 20240715125853.png#invert|400]]
The diagram you provided illustrates the mechanics of a prosthetic hand utilizing a block and tackle system to actuate finger movements via a single motor. This mechanism, commonly used in prosthetic designs, employs cables and pulleys to facilitate complex movements with fewer motors, significantly reducing weight and complexity. Here’s how it works:

### Components and Mechanics:
1. **Motor and Tendon**:
   - The motor generates a traction force $F_z$ which is transmitted through a tendon connected to the bottom right. This force initiates the movement of the entire system.
2. **Pulley System**:
   - The pulley $P$ receives the force from the motor and moves downward. Due to the mechanics of the pulley system, the force exerted on the upper tendon (above the pulley) doubles ($F_l = 2F_z$).
   - The system effectively increases the force output while using the same initial motor force, enabling more substantial grip strength without additional motor power.
3. **Tendon Movement**:
   - The travel distance of the upper tendon and the movement of the pulley $P$ is half the movement of the lower tendon ($S_l = \frac{S_z}{2}$), which indicates a mechanical advantage, reducing the distance the motor needs to move to achieve the desired finger positioning.
4. **Force Calculation**:
   - As the tendon wraps around the pulley, the traction force $F_p$ exerted by the pulley can be calculated, where $T_r$ is the torque around the pulley, derived from the formula $T_r = F_p \times \frac{d}{2}$. Here, $d$ represents the diameter of the pulley, which affects the torque exerted by the force at the pulley.
### Practical Application:
- **Finger Actuation**:
  - This setup controls multiple fingers (Index, Middle, Ring, Little) through a single motor by varying the distribution of force through the tendon-pulley system.
  - The fingers are connected to the cables which, when actuated by the motor and pulley system, result in precise and controlled finger movements.
- **Efficiency**:
  - This system allows for a compact, efficient design that can be scaled and adapted for various hand sizes and strengths, suitable for personalizing prosthetic devices to fit individual needs.
This type of underactuated mechanism is beneficial for prosthetic hands, as it allows for complex finger movements with minimal motors, reducing weight, cost, and maintenance while providing adequate force and precision for daily tasks.
## Prosthetic Control Strategies
### Basic Myoelectric Control
- [[s40137-013-0044-8.pdf]]
- **Early Myoelectric Hands**: Initially, prosthetic hands were controlled using electromyography (EMG) signals derived from one or two muscle groups. These signals were used to switch the prosthetic device between different states:
    - **Slight Contraction**: Triggers the prosthetic hand to open.
    - **Strong Contraction**: Causes the hand to close.
    - **No Muscle Activity**: Stops all movement, halting the device.
- **Two Electrodes System**: Placing electrodes on opposing muscle groups allowed for more straightforward and direct control of opening and closing actions of the prosthetic hand.
![[Pasted image 20240716115811.png#invert|400]]
### **Advanced Myoelectric Control**
![[Pasted image 20240716115832.png#invert|400]]
- **Pattern Recognition**: This method involves extracting features from EMG signals and classifying them to control the prosthetic. This approach allows for more complex functions than simple open/close commands.
    - **Feature Extraction**: Involves analyzing EMG signals to identify distinctive patterns that correspond to specific intended movements.
    - **Classification**: Algorithms classify these patterns to determine the action the user is attempting to execute. However, traditionally, this could only activate one function at a time.
    - **Regression**: Unlike classification, regression techniques map EMG signals directly to continuous control signals, allowing for more dynamic and natural movement. This method can identify and execute multiple movements simultaneously, offering a more intuitive control experience.
![[Pasted image 20240716115908.png#invert|400]]
### **Two-Channel Myoelectric Control**
- This system uses two electrodes placed over the main flexor and extensor muscles of the forearm. These muscles typically control the flexing and extending movements of an able-bodied individual's hand.
- **Training and Adaptation**: Users of these prosthetic systems must undergo training to learn how to activate these muscles purposefully. This training is crucial as they have to learn to generate reliable EMG signals without the physical movement of a hand, focusing solely on muscle contractions.
### **Practical Applications and Challenges**
- **Improved Functionality**: Advanced myoelectric controls provide users with capabilities closer to those of a natural limb, including more precise and varied movements.
- **User Training**: Effective use of these prosthetics requires significant training to fine-tune muscle control and adapt to the prosthetic’s feedback and handling.
- **Technological Limitations**: Despite advancements, challenges such as signal inconsistency due to factors like muscle fatigue or external interference still persist, affecting the reliability and functionality of the prosthetic.
### EMG grasp control
This involves various control schemes that allow users to operate the prosthetic hand through muscle contractions detected via EMG electrodes.
1. **Single Contraction**: Involves contracting a single muscle to trigger an action like opening or closing the hand.
2. **Co-contraction**: Simultaneous contraction of two muscles, often used to switch between different modes or functions of the prosthetic hand.
3. **Safety Co-contraction**: A specific sequence where a co-contraction is followed by a single contraction to confirm and execute a command, enhancing safety by reducing accidental activations.
4. **Impulse Switching**: A short, sharp contraction of a muscle to switch functions or activate the device.
5. **Hold-Open**: A stronger contraction performed after the prosthetic hand has completed its motion, typically used to keep the hand open.
6. **Four-Channel Control**: Advanced control strategy where a single electrode can control two degrees of freedom (DoF); different intensities and types of muscle contractions distinguish between the controls.
### **Commercially Available Myoelectric Hands**
- **Michelangelo by Otto Bock**: Uses co-contraction to switch between different grasps, with the option to define additional muscle signals for more grasp options.
- **Bebionic Hand v3 by Otto Bock**: Similar to Michelangelo, it uses co-contraction and hold-open techniques, with an additional button for accessing more grasp types.
- **i-limb ultra by Össur**: Incorporates co-contraction, hold-open, and repeated single contraction patterns, with gesture control to change grasps based on arm motion direction.
- **HERO arm by Open Bionics**: Utilizes hold-open to switch between grasps, also featuring a button for more options.
- **VINCENTevolution by Vincent Systems**: Allows switching through co-contraction or single contractions followed by a longer signal for direction, enabling various predefined grasps.
- **TASKA Hand by TASKA Prosthetics**: Offers personalized EMG patterns and additional controls via buttons and a Bluetooth-connected app for numerous grasp options.
### **Pattern Recognition and Advanced Control**
- Advanced prosthetic systems are now incorporating myoelectric pattern recognition, allowing for more nuanced control of the prosthesis with multiple electrodes (up to eight). This system captures a broader range of muscle activities, enabling more personalized and precise control over the prosthetic functions.
- **Individual Training**: These systems require individual training for each user to accurately map muscle activity patterns to prosthetic controls.
- **User-Specific Limitations**: The effectiveness of pattern recognition and the ability to control multiple muscles independently can be significantly impacted by the specific conditions of the amputation.
### Semi-Autonomous Grasping
The concept of **semi-autonomous grasping** in modern prosthetic hands aims to bridge the gap between the advanced capabilities of prosthetic devices and the user-friendly control systems needed for their effective use. Traditional prosthetic hands often require manual selection of grasp types, which can be cumbersome and unintuitive for the user. The idea behind semi-autonomous grasping is to enhance the control system with additional sources of information, allowing the prosthetic hand to make some decisions autonomously while the user supervises and guides these decisions. Here’s how this approach is implemented and utilized:
#### Components of the Semi-Autonomous System
1. **Semi-Autonomous Controller**: This is usually implemented on a computing platform like a standard laptop. It processes inputs from various sensors and decides the most appropriate grasp type based on the context it perceives.
2. **Augmented Reality (AR) Glasses**: These are equipped with stereo cameras that provide a 3D view of the environment directly to the controller. AR glasses enhance the user's interaction with the environment by overlaying digital information onto the real world, which can be particularly useful in assisting the user in selecting and controlling grasp types.
3. **Myoelectric Interface**: Typically, a two-channel system is used. This interface reads EMG signals from the user's residual limb muscles. These signals are interpreted by the controller to determine user intent regarding movement and grasp type.
4. **SmartHand Multi-Grasp Prosthesis**: This is a sophisticated prosthetic hand capable of performing multiple grasp types. It responds to commands from the semi-autonomous controller based on the data received and processed.
![[Pasted image 20240716121148.png#invert|400]]
- [[Markovic_2014_J._Neural_Eng._11_046001.pdf]]
- [[Markovic_2015_J._Neural_Eng._12_066022.pdf]]
### Advanced Sensing and Interaction
- **Stereo Vision**: Utilizes a pair of cameras (stereoscopic vision) to gain depth perception, much like human binocular vision, which helps in accurately determining the position and size of objects in the environment.
- **Augmented Reality with Embedded Cameras**: Newer versions might include an RGBD (Red, Green, Blue, Depth) camera combined with an inertial sensor on the prosthesis. RGBD cameras provide color images along with depth information, which are crucial for precise object recognition and spatial positioning.
- **Inertial Sensors**: These measure the orientation and movements of the prosthesis, helping the system to better understand the positioning of the hand relative to objects to be grasped.
### Operational Mechanism
- The controller uses the visual and sensory data to autonomously determine the most suitable grasp type for an object.
- The user can supervise and make adjustments if necessary, either through direct EMG input or through other control interfaces like buttons or voice commands.
- The system aims to reduce the cognitive load on the user by handling complex decisions about grasping, while still allowing for user override or confirmation.
### Finite State Machine
![[Pasted image 20240716121412.png#invert|400]]
The control system operates as a finite state machine, meaning it transitions between predefined states based on user inputs and sensor data. These transitions are influenced by muscle activity signals detected through myoelectric control (from the user's residual limb), allowing the system to interpret user intent regarding hand movements.
#### State Transitions
1. **Start/Initial State**: The system activates when the user focuses on an object, potentially using gaze direction which can be detected via AR glasses equipped with eye-tracking or a similar technology.
2. **Object Targeting Phase**: Once the object is targeted, the system uses computer vision to recognize and overlay the object with visual cues in the AR glasses. This indicates to the user that the object has been successfully identified.
3. **3D Object Modeling and Analysis**:
    - The system creates a geometrical model of the object using computer vision.
    - It analyzes the object’s size and shape to estimate the necessary grasp type and the size of the aperture (opening) required by the prosthetic hand to grasp the object.
4. **Prosthesis Preshaping Phase**:
    - Based on the analysis, the system commands the prosthetic hand to preshape, preparing it for the specific grasp type identified.
    - This preshape configuration is visually presented to the user through an icon displayed in the AR glasses. The user also sees a virtual representation of the aperture size needed next to the object.
5. **User Input for Final Adjustment**:
    - At this stage, the user can adjust the preshape aperture using EMG commands if the initial estimation isn’t satisfactory.
    - These adjustments are made by interpreting high intensity or low intensity flexion or extension commands from the EMG interface, denoted as HE (High Extension), LE (Low Extension), HF (High Flexion), and LF (Low Flexion).
6. **Prosthesis Opening and Closing**:
    - Depending on the user's final input and confirmation, the prosthetic hand either opens or closes around the object to execute the grasp.
7. **Manipulation Phase**: Once the object is securely grasped, the user can manipulate it as needed.
#### Key Features of the FSM-based Control System
- **Flexibility**: The system allows for user intervention at multiple points, ensuring that the automatic estimations can be overridden or adjusted for better control and comfort.
- **Integration with AR**: This provides an intuitive interface that enhances user interaction with the prosthetic device, making the grasping process more natural and efficient.
- **Myoelectric Inputs**: These serve as the primary user interface, translating muscle movements into specific commands that guide the prosthetic’s actions.
### Experiments to evaluate different Control Strategies
- **Participants**: 13 able-bodied subjects.
- **Prosthesis Used**: SmartHand, a sophisticated prosthetic device.
- **Task**: Simple grasping tasks that included reaching an object at location B, grasping it, transporting it to location C, and then releasing it.
- **Trials**: A total of 1560 trials were conducted using 20 different objects.
- **Control Modes**:
    - **AUTO-AR**: Fully automatic control with AR feedback where subjects could not make corrections.
    - **SEMI-AR**: Semi-automatic control with AR feedback allowing subjects to make corrections.
    - **SEMI-AR-RE**: Similar to SEMI-AR but with random errors introduced into the vision system to simulate practical challenges.
    - **SEMI-VIS-RE**: Semi-automatic control without AR feedback but with random errors introduced.
#### Results and Findings
- **Performance**: The experiments demonstrated that allowing user corrections (SEMI-AR mode) significantly improved system performance (10-16% improvement) compared to fully automatic control (AUTO-AR).
- **User Interaction**: Subjects intervened in about 25% of the trials, adjusting grasp type or aperture size to better fit the task requirements.
- **Time Efficiency**: The average time to complete the task was shorter in AUTO-AR mode (2.7 seconds) compared to SEMI-AR mode (3.47 seconds), likely due to the time taken for manual adjustments.
#### Additional Insights
- **MYO-PACE**: A concept described as myoelectric semi-autonomous grasp control that integrates an infrared stereovision system for environmental perception, an IMU for proprioception, and visual feedback via AR glasses. The EMG control is sophisticated, allowing detailed decomposition of multi-electrode readings to estimate user intent and autonomously control hand preshaping.
- **KIT's Deep Learning Application**: The KIT experiment involves training a deep convolutional neural network (DCNN) with images from a camera integrated into the prosthetic hand for object recognition. This system autonomously selects finger pre-shaping and grasp force based on the recognized object, which is a significant advancement in making prosthetic hands more responsive and intuitive.
## Research Challenges
### Hand Size and Personalization
- **Challenge**: Accommodating various hand sizes (female, male, child) in prosthetic design is crucial for ensuring a good fit and comfort. Prostheses need to be scalable or adjustable to match the physical dimensions of the user’s remaining limb or the contralateral hand.
- **Implications**: This requires advanced fabrication techniques such as 3D printing or modular design elements that can be customized to individual specifications without compromising the integrity and functionality of the prosthetic hand.
### Versatile Control
- **Challenge**: Prosthetic hands must effectively use all degrees of freedom (DoF) provided by the design to mimic natural hand movements. The control system must be efficient, allowing users to perform complex tasks with minimal effort and without a steep learning curve.
- **Implications**: Developing control algorithms that can interpret the user’s intent from limited input data, such as EMG signals, is vital. These systems must be able to handle multiple functions and transitions smoothly between different types of grips and hand positions.
### Robustness and Fault Classification
- **Challenge**: Ensuring that the prosthetic hand operates reliably under various conditions and accurately interprets the user's intentions with a low rate of misclassification or fault.
- **Implications**: Advanced signal processing and machine learning techniques are required to enhance the accuracy and reliability of myoelectric control systems. This also includes improving the hardware's resilience to environmental factors and daily wear and tear.
### Interfaces to the User
- **Challenge**: Creating robust interfaces that can decode complex biosignals effectively and consistently is critical for a responsive prosthetic hand. The interface must be capable of handling signal variability due to changes in the user’s physiological condition, like muscle fatigue or emotional stress.
- **Implications**: This involves integrating advanced sensor technology and developing sophisticated algorithms capable of adapting to changes in signal quality and patterns. Improving electrode design and placement can also enhance signal stability and accuracy.
### Feedback Mechanisms
- **Challenge**: Providing meaningful feedback to the user about the status and interaction of the prosthetic hand with objects. This feedback can be visual (through displays or augmented reality), haptic (through tactile feedback mechanisms), or auditory.
- **Implications**: Integrating sensory feedback systems into the prosthesis can significantly improve the user's control and perception of the prosthetic hand. This requires the development of sensors that can detect touch, pressure, and temperature, and the implementation of systems that can convey this information back to the user in an intuitive manner.
