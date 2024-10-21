> [!abstract] Definition
> Exoskeletons are sophisticated devices designed to interact with the human body, enhancing its functions or compensating for weaknesses or injuries.

## Classification Criteria for Exoskeletons

### Kinematic Equivalence:
   - **End-Point Based**: Focuses on the position and orientation of the end effector (like a hand or foot attached to the exoskeleton).
   - **Kinematically Equivalent**: The exoskeleton's joints and segments mirror those of the human body to replicate natural movements closely.
   - **Soft**: Utilizes flexible materials and mechanisms to conform more closely to the natural movements and structure of the human body.
   - This particular classification relates to how closely an exoskeleton replicates the human body's range of motion and redundancy in its joints.
	- **Kinematic Redundancy**: Refers to having more degrees of freedom than necessary for basic tasks, mirroring the human body's ability to perform a wide range of movements using various paths and methods. This redundancy allows for versatile motion and increases the adaptability of the exoskeleton to different tasks and environments.
	- **Example**: The human arm-shoulder system, which exhibits high redundancy, allowing for the complex and highly variable movements seen in tasks ranging from typing to throwing a ball.
### Segment/Joint:
   - Classifications based on which part of the body the exoskeleton is designed for, such as full body, leg, arm, knee, or hand.
### Degree of Freedom (DoF):
   - Describes the number of independent movements a device can make. Higher degrees of freedom allow for more complex and versatile movements.
### Link Configuration:
   - **Serial**: Joints are connected in a series, one after another.
   - **Parallel**: Joints and limbs have concurrent connections, often resulting in more robust and stable configurations.
   - **Hybrid**: Combines elements of both serial and parallel configurations.
### Actuation:
- **Electric (72%)**: Most common due to their ease of control, efficiency, and quiet operation.
- **Hydraulic (5%)**: Used for applications requiring high force and precise control but are less common due to their bulk and maintenance requirements.
- **Pneumatic (20%)**: Frequently used where lighter and more compliant actuation systems are needed, though they can be less precise.
- **Other (3%)**: Includes newer or less common technologies.
### **Power Transmission**:
- **Motor - Cable Drive (26%)**: Uses cables to transfer force, allowing for flexible configurations and reduced weight.
- **Motor - Gear Drive (21%)**: Utilizes gears to increase torque output, suitable for compact designs needing high power.
- **Motor - Other (25%)**: Could include belts, chains, or direct drives, each with unique advantages for specific applications.
- **Pneumatic - Pneumatic (14%)** and **Pneumatic - Other (8%)**: Pneumatic systems using air pressure, often in more specialized applications.
- **Hydraulic - Oil (3%)**: Employs hydraulic fluid to transmit power, offering high force and precise control.
### Control Methods:
   - **Impedance**: Modifies mechanical impedance to adapt to changes in the environment.
   - **Force**: Direct force control for robust interactions.
   - **Fuzzy-Neuro**: Advanced algorithms that combine fuzzy logic and neural networks for adaptive control.
### Interface to Human Body:
   - **EMG (Electromyography)**: Interfaces that read electrical activity from muscles to control the device.
   - **FMG (Force Myography)**: Uses pressure sensors over muscles to detect changes in muscle shape.
   - **Mechanical**: Direct mechanical linkage to the human body.
### Application Areas:
#### Rehabilitation
- **Purpose**: These exoskeletons are designed to aid in the rehabilitation process of patients recovering from injuries or medical conditions affecting their mobility.
- **Key Features**:
    - **Monitoring Health and Progress**: Capable of gathering data to assess the patient's recovery over time.
    - **Adaptability**: Ability to adjust its support as the patient's condition improves or changes.
    - **Torque and Trajectories**: Provides sufficient torque to fully assist or perform movements if the patient is unable to do so themselves, with precise control over the movement paths.
    - **Power Source**: Can be either tethered (connected to a power source) or battery-operated, depending on the need for mobility and the duration of use.
#### Augmentation
- **Purpose**: Augmentation exoskeletons are used to enhance the physical capabilities of the user, often in industrial or military applications.
- **Key Features**:
    - **Natural Motion Follow**: Designed to complement and enhance the user’s natural movements without obstructive interference.
    - **Torque**: Provides additional force to assist in tasks, reducing the user’s physical strain.
    - **Actuation Speed**: Fast actuation to keep pace with dynamic movements.
    - **Kinematic Compatibility and Battery**: Must have good kinematic alignment with the user’s body and sufficient battery capacity for extended use without frequent recharges.
#### Substitution
- **Purpose**: Aimed at completely substituting the motor functionality for individuals with severe disabilities or to perform specific tasks that are otherwise humanly impossible.
- **Key Features**:
    - **Daily Activities**: Enables users to perform daily activities independently.
    - **Torque and Trajectories**: Provides strong enough torque to support or carry the user’s full weight and can generate complete movement trajectories needed for various activities.
    - **Battery Capacity**: Requires a robust battery system to ensure prolonged use without frequent recharging.
### Assistance Type
- What comes after one ensures the kinematic compatibility?
- Kinematic compatibility is easy to observe 
- However, while being compatible, the exoskeletons may fail to provide assistance and can instead make the movements more difficult
#### Full Mobilization Exoskeletons
**Purpose**: These exoskeletons are designed primarily for individuals who have experienced severe loss of motor control, such as those with spinal cord injuries (SCI).
- **Functionality**: They assist or entirely take over the movement of body parts. This includes enabling users who may not be able to stand or walk on their own to perform these actions with the exoskeleton’s support.
- **Kinematic Compatibility**: While these devices need to be kinematically compatible, the primary focus is on providing sufficient power to execute movements. This means they must align well with the user’s body to move effectively without causing harm or discomfort.
- **Torque Requirements**: Full mobilization exoskeletons require high torque capabilities because they need to supply all the necessary force for movement. This can involve powerful motors and robust mechanical systems.
- **Visibility of Benefits**: The benefits of full mobilization exoskeletons are usually immediately apparent and profound, as they enable mobility where it was previously impossible, thereby significantly enhancing the user's quality of life.
#### Partial Assistance Exoskeletons
**Purpose**: These are designed for individuals with less severe impairments, such as diminished strength or endurance due to aging or minor disabilities. They are also used by healthy individuals in situations requiring enhanced endurance or strength.
- **Functionality**: Instead of taking over complete movement, these exoskeletons provide supplemental assistance. This could mean helping to stabilize movements, reduce load on particular muscles, or enhance the endurance of the user.
- **Kinematic Compatibility**: For these devices, ensuring kinematic compatibility is crucial because any misalignment can make the device counterproductive, either by failing to reduce effort or by increasing the energy required for movement.
- **Torque Requirements**: Compared to full mobilization exoskeletons, the torque requirements are generally lower. These devices need only supplement the user’s existing capabilities rather than replace them entirely.
- **Visibility of Benefits**: The benefits of partial assistance exoskeletons might not be as immediately obvious as those of full mobilization systems. Determining effectiveness may require measuring the user's energy expenditure to ascertain whether the device truly makes movements easier or more efficient.
## Four Different Types of Exoskeletons
![[Pasted image 20240710103807.png#invert|800]]
### 1. End-Point Based Exoskeletons
**Description**: These exoskeletons connect only at the extremities of the human limb, such as the hand or foot. They do not follow the human kinematic chain.
- **Advantages**:
    - Independent of the human kinematic chain, allowing flexibility in design and use.
- **Disadvantages**:
    - Can lead to inaccurate application of torques.
    - Parasitic torques may occur, which can strain or damage the musculoskeletal system.
- **Example Applications**:
    - Used for transferring additional loads directly to the ground.
    - Suitable for telemanipulation tasks in remote or hazardous environments.
- **Example Devices**: EXO-UL7, BLEEX, NextExo.
### 2. Kinematically Equivalent Exoskeletons
**Description**: These devices are closely integrated with the user's limbs, matching each human joint with an exoskeleton joint.
- **Advantages**:
    - Simple structure that generally requires less space to operate.
    - Aligns closely with human anatomy, which can enhance movement accuracy.
- **Disadvantages**:
    - Requires manual alignment to the user’s joint axes, which can be time-consuming and must be precise to avoid discomfort.
    - Potential for micro-misalignments during movement, which may affect performance or comfort.
- **Example Applications**:
    - Substitution of lost functionalities, such as for stroke survivors.
    - Rehabilitation to assist recovery from injuries.
- **Example Devices**: [[NEUROExos]], Ekso Bionic Suit, HAL, KIT-EXO.
### 3. Kinematically Different Exoskeletons
**Description**: These also attach to every segment of the limb but use multiple joints for each human joint, providing greater flexibility and potentially more precise control.
- **Advantages**:
    - Allows for automatic alignment with the user’s limbs, enhancing comfort and reducing setup time.
    - Generally offers good wearability due to a more adaptive design.
- **Disadvantages**:
    - More complex structure, which may involve more maintenance and higher costs.
    - Requires more space than simpler designs, which could limit its usability in confined spaces.
- **Example Applications**:
    - Extensively used in rehabilitation for more comprehensive support.
    - Strength augmentation for physically demanding tasks to reduce fatigue and injury risk.
- **Example Devices**: [[HARMONY]], iT-Knee, UtahKnee, [[CAREX]]
### 4. Soft Exosuits
**Description**: Made from soft fabrics and actuated typically via Bowden cables, these suits offer support without rigid structures, focusing on enhancing basic movements.
- **Advantages**:
    - Extremely comfortable due to the soft materials and flexible design.
    - Compact and lightweight, making them easy to wear under clothing.
- **Disadvantages**:
    - The softer structure may lead to less precise torque application, which can be a concern in tasks requiring high accuracy.
    - Risk of generating parasitic torques that could potentially lead to discomfort or injury.
- **Example Applications**:
    - Gait assistance for patients with muscle weakness, aiding in everyday mobility.
    - Augmentation of healthy users in activities that require extended physical effort.
- **Example Devices**: Harvard Soft Exosuit, Myosuit, [[CRUX]]

## Kinematic Incompatibilities (Misalignments)
- The critical issue of kinematic incompatibilities or misalignments between a wearable exoskeleton and the human limb it is intended to assist or enhance. 
- **Kinematic incompatibility** refers to the discrepancies or misalignments that occur when the movement mechanics of the exoskeleton do not perfectly align with those of the human limb. 
### Reasons
1. **Variability Between Subjects**:
   - **Bone Lengths**: Different individuals have varying lengths of bones, which can affect how well an exoskeleton fits.
   - **Distances Between Rotation Axes**: The points around which joints rotate can differ significantly between people.
   - **Orientations of Rotation Axes**: The angle at which joints operate can vary, affecting how movements are performed.
2. **Variability Within Individual Subjects During Movement**:
   - **Movement of Joint Centers of Rotation**: As joints move, the actual center of rotation can shift, which is not always accounted for in the rigid structures of exoskeletons.
3. **Unpredictability of Joint Axis Locations and Body Segment Sizes**:
   - These variations can lead to challenges in ensuring that the exoskeleton aligns properly with the user’s limbs throughout different movements.
### Effects of Misalignments
Misalignments can have several detrimental effects on the performance of the exoskeleton and the comfort and safety of the user:
1. **Relative Movement**:
   - If the exoskeleton does not align perfectly with the user's limb, there can be unwanted relative movements between the device and the limb. This can lead to inefficiencies in force transmission and potential disturbances in the intended movement patterns.
2. **Shear and Pressure Forces**:
   - Misalignments can create additional forces that are not aligned with the normal forces exerted during movement. These forces, particularly shear forces, can lead to discomfort, skin abrasions, or even more serious injuries at the points where the exoskeleton contacts the body.
   - Pressure forces can be equally problematic, leading to reduced blood circulation, nerve compression, or tissue damage under prolonged use.
### Macro-Misalignments
**Macro-misalignments** refer to large-scale discrepancies between the joint structures and degrees of freedom (DoFs) of human limbs and those of the exoskeleton.
- **Causes**:
  - This type of misalignment typically occurs when the exoskeleton's joints are oversimplified—meaning they have fewer DoFs compared to the human joints they are meant to assist or augment.
  - An example of this can be seen in upper limb exoskeletons designed with only seven DoFs, whereas the human shoulder complex involves a significantly more complex arrangement that allows for a wider range of motion.
- **Effects**:
  - In **end-point based exoskeletons** (those that only connect at the limb extremities), macro-misalignments might not significantly impact the device's function because these exoskeletons do not rely on precise joint articulation to operate effectively.
  - However, in **wearable exoskeletons** that need to mimic or enhance the wearer's movements, such misalignments can severely restrict the natural movement range and the shared workspace, thereby reducing effectiveness and comfort.
### Micro-Misalignments
**Micro-misalignments** are smaller-scale inaccuracies that occur when the exoskeleton joints fail to exactly follow the human joints' movements, particularly the instantaneous center of rotation (ICR).
- **Causes**:
  - These occur even if the exoskeleton matches the human joints in terms of the number of DoFs. The misalignments are typically due to slight offsets in joint alignment or the exact movement path, such as an offset in the rotation axis of the elbow.
- **Effects**:
  - These misalignments lead to the creation of unwanted interaction forces—such as shear forces—which occur between the attachment points of the exoskeleton and the human limb. These forces can lead to discomfort and even injury with prolonged use.
### Design Considerations for Exoskeletons
Given the challenges posed by both macro- and micro-misalignments, designing exoskeletons that can adapt to individual anatomical differences while maintaining functionality and comfort is crucial. Here are some strategies:
- **Redundant Mechanisms**:
  - Incorporating additional DoFs beyond what is minimally necessary can allow the exoskeleton to adjust its movement paths for better alignment with the human user.
- **Compliant Mechanisms**:
  - These mechanisms can absorb or adjust to minor misalignments through materials or designs that accommodate or adapt to variations in human anatomy and movement. This can include elastic or semi-rigid components that conform more closely to the user's body.
- **Adjustability and Customization**:
  - Designing exoskeletons with adjustable joints or modifiable settings that can be tailored to the specific measurements and movement styles of the user can help minimize the effects of misalignments.
- **Trade-offs**:
  - While redundant and compliant mechanisms can greatly enhance comfort and usability, they may also increase the weight and complexity of the exoskeleton, potentially impacting its efficiency and wearability.
## Example: Negative Effect of Added Mass to Legs
![[Pasted image 20240710104642.png#invert|400]]
- **Y-axis (Net Metabolic Rate in W/kg)**: This axis represents the metabolic rate, which is the rate at which a person burns calories per kilogram of body weight. It's a measure of how much energy the body needs to perform activities, expressed in watts per kilogram.
- **X-axis (Load in kg)**: This shows the additional weight (load) attached to different parts of the leg—foot, shank, thigh, and waist.
- **Data Lines**: Each line represents a different part of the leg where weight is added. The increase in the net metabolic rate with increasing load is shown for each location.
### Key Observations from the Graph
1. **Load Impact by Location**:
    - **Foot**: Adding weight to the foot shows a significant increase in energy expenditure compared to other parts. Even a small amount of added weight at the foot requires much more energy, as indicated by the steep slope of the line.
    - **Shank, Thigh, and Waist**: The impact of added weight at these locations is progressively less severe than at the foot, but still notable. The shank and thigh exhibit similar patterns, with the waist being slightly better in terms of metabolic cost.
2. **Metabolic Rate Increases**:
    - The red box highlights the effect of adding 1 kg to the foot, increasing the metabolic rate from about 2.35 W/kg to 2.55 W/kg, which corresponds to an 8% increase in energy expenditure.
### Explanation of the Negative Effect of Added Mass to Legs
- **Increased Inertia**: Adding mass to the legs, especially at the extremities like the feet, significantly increases the inertia. This means the legs require more force and energy to move and stop moving, as inertia relates to the mass's resistance to changes in its state of motion.
- **More Power Needed**: The increased inertia necessitates more muscular effort to accelerate and decelerate the limbs during walking or running, thus increasing the overall energy required for movement.
- **Metabolic Rate Increase**: The metabolic rate is directly influenced by the amount of energy the body must expend to overcome these challenges. Higher metabolic rates indicate more strenuous activity, which can lead to quicker fatigue.
### Implications for Exoskeleton Design
- **Lightweight Construction**: To minimize the negative effects of added mass, exoskeletons should be as lightweight as possible, especially in components attached to the feet or other moving parts of the body.
- **Optimal Mass Distribution**: Designers need to consider where to place additional mass to minimize its impact on the wearer's energy expenditure. Placing mass closer to the body's core (e.g., waist) rather than the extremities can help reduce the metabolic cost.
## Exoskeleton Evaluation – Biofeedback
The evaluation of exoskeletons in terms of their effectiveness and user impact often involves biofeedback metrics like muscle activity, heart rate, and analysis of respiratory gases. Each of these metrics provides insights into how much effort a user is exerting while using the exoskeleton and can indicate the device's effectiveness or areas for improvement.
### Muscle Activity
- **What it Measures**: Muscle activity is directly measured through techniques like electromyography (EMG), which records the electrical activity produced by skeletal muscles. Lower muscle activity while using the exoskeleton suggests that the device is effectively reducing the user's effort.
- **Challenges**:
    - **Limited Coverage**: Typically, only a few muscles are measured directly due to practical limitations. This can lead to incomplete data, as the human body might compensate by increasing effort in muscles that are not being monitored.
    - **Co-contraction**: This refers to the simultaneous contraction of agonist and antagonist muscles, which might increase despite reduced activity in the targeted muscles, thus complicating the interpretation of muscle activity data.
![[Pasted image 20240710105451.png#invert|400]]
- Shows how different spring stiffness in an exoskeleton affects metabolic expenditure and muscle activity. Notably, a specific stiffness (around 100 N m rad^-1) is shown to minimize metabolic rate, indicating an optimal balance that potentially reduces user effort.
### Heart Rate
- **What it Measures**: Heart rate monitoring is another common method to infer user effort. Generally, a lower heart rate during task performance would indicate less physical strain.
- **Challenges**:
    - **Confounding Factors**: Heart rate can be influenced by a variety of factors beyond physical exertion, such as emotional stress, environmental conditions, and individual health status. This variability can make heart rate a less reliable indicator of physical effort alone.
![[Pasted image 20240710105514.png#invert|200]]
- Illustrates the relationship between exercise intensity and heart rate. This graph could be used to study how using an exoskeleton affects heart rate at various levels of physical activity, although the specific context for exoskeleton use isn't provided in this graph alone.
### Respiratory Gases
- **How it is Measured**: This method involves analyzing the gases that the user inhales and exhales, specifically looking at oxygen consumption.
- **Process**:
    - **Measurement of Oxygen Uptake**: The difference between the amount of oxygen inhaled and the amount exhaled during respiration is measured. A decrease in exhaled oxygen relative to inhaled oxygen indicates that more oxygen is being utilized by the body, which implies greater metabolic activity and energy expenditure.
- **Reliability**: Analyzing respiratory gases is considered more reliable than the previous methods because it provides a direct measure of metabolic rate. However, it is more complex and can be tedious, requiring specific equipment and controlled testing environments.
## Design Process for Arm Exoskeletons
### 1. **Definition of Requirements**
- **Task Analysis**: Initially, the tasks for which the exoskeleton is intended must be clearly defined. For arm exoskeletons, this often involves analyzing movements required in activities of daily living (ADLs). The process for designing lower limb exoskeletons follows a similar structure but focuses on different tasks relevant to leg function.
- **Motion Recording**: Movements of the human arm are recorded using advanced motion capture systems (like Vicon with 10 cameras) to gather precise data on how the arm moves during various activities. This data is crucial for designing joints and mechanisms that can mimic or support these movements.
- **Alternative Data Sources**: Besides direct motion capture, designers can also use existing motion databases or literature to inform their designs, especially if certain specific movements have been well-documented previously.
### 2. **Motion Analysis**
- **Activities Selection**: The specific activities selected for analysis often include general reaching tasks, functional tasks like eating and drinking, and hygiene-related tasks. These categories cover a broad range of essential daily activities that an exoskeleton might need to assist with.
- **Torque Calculation**: Understanding the forces involved in arm movements is crucial. Torque calculations can be performed using software like Cosmos/Motion in Solidworks, or through analytical approaches using tools like Autolev or Online Dynamics.
### 3. **Human Motion Analysis**
- **Joint Angle and Torque Distribution**: Analyzing the statistical distribution of joint angles and torques during these activities helps in understanding the natural range and force with which human joints operate. This step is essential for ensuring that the exoskeleton can accommodate and support these natural movements without causing strain or discomfort.
### 4. **Joint Kinematic and Dynamic Analysis**
- **Range of Motion (ROM)**: Identifying the range of motion required for each joint in the arm ensures that the exoskeleton can fully support or enhance all necessary movements.
- **Root Mean Square (RMS)**: This statistical measure can be used to analyze the variability and typical values of joint angles and torques, providing a robust foundation for design decisions.
- **Workspace and Torque Requirements**: From the pilot study on ADL, designers generate data tables that detail the workspace (the volume of space within which all necessary movements occur) and torque requirements for the exoskeleton. This data highlights critical areas like elbow flexion-extension and forearm prono-supination, where a significant range of motion (up to 150°) is necessary.
### Joint Configurations
- Three exoskeleton configurations that achieve rotation about the long axis of a limb segment
![[Pasted image 20240710110353.png#invert|300]]
#### Configuration a) Proximally Placed Single DOF
- **Description**: This configuration places a single degree of freedom (DOF) joint proximally (near the base) along the limb segment. The joint’s axis of rotation aligns with the limb's anatomical axis.
- **Advantages**:
  - Simplicity in design allows for easier integration and maintenance.
  - Placing heavy components like bearings and actuators close to the body reduces the inertia effects, which can lower power consumption.
- **Disadvantages**:
  - Placement may cause interference during certain movements, such as shoulder abduction, where the natural movement of the shoulder can be restricted by the exoskeleton’s structure.
#### Configuration b) Circumferentially Placed Single DOF
- **Description**: This setup positions the single-DOF joint axially between the ends of the limb segment. It uses a bearing whose minimum radius is larger than the limb's maximum radius to allow free rotation around the segment axis.
- **Advantages**:
  - Reduces the human-machine interference found in configuration a) by positioning the joint away from areas where the limb has extensive interaction with the exoskeleton.
- **Disadvantages**:
  - Full 360-degree bearings can interfere with the torso, particularly when the arm is at rest or during movements bringing the arm close to the body.
  - Modifications like using a partial bearing attached to the proximal link of the exoskeleton can alleviate this interference but may compromise the range of motion.
#### Configuration c) Three Parallel, Non-Collinear DOFs
- **Description**: Involves a primary axis that is displaced away from the anatomical axis, supplemented by at least two additional non-collinear axes to correct for the displacement and ensure accurate rotation.
- **Advantages**:
  - Avoids the interference issues noted in configurations a) and b) by laterally displacing the joint axis from the limb's natural axis of rotation.
- **Disadvantages**:
  - Adds significant complexity and weight to the exoskeleton design because of the additional joints required to maintain proper rotation dynamics.
  - This complexity can increase the cost, maintenance requirements, and reduce reliability if not designed meticulously.
### Strength-to-Weight and Energy-to-Weight Ratios
- **Importance**: These ratios are crucial for the feasibility and functionality of exoskeletons. The strength-to-weight ratio relates to the amount of weight a material or structure can support relative to its weight, crucial for making exoskeletons both strong and lightweight. The energy-to-weight ratio concerns the amount of energy a power supply can deliver relative to its weight, important for the autonomy and mobility of the exoskeleton.
- **Challenges**: Currently, the available materials, electric motors, and power supplies may not offer the optimal balance necessary for developing mobile, partial-body upper limb exoskeletons. These limitations can affect the exoskeleton's performance, making it either too heavy or insufficiently powerful for extended use without frequent recharges.
- **Implications**: Because of these limitations, there is often a need to design full-body exoskeletons that can distribute the weight of power supplies, onboard controllers, and other hardware more effectively across the user's body. This distribution helps manage the weight and maintain balance, although it complicates the design and increases the overall size of the exoskeleton.
### Safety Considerations
- **Levels of Safety**:
  - **Mechanical Safety**: Ensures that the exoskeleton does not cause physical harm through sharp edges, pinch points, or uncontrolled movements.
  - **Hardware Safety**: Involves reliable and fail-safe mechanical components, robust electrical systems, and emergency stop mechanisms.
  - **Software Safety**: Includes software that can handle errors gracefully, manage unexpected inputs without crashing, and ensure reliable control over all operations.
- **Self-Collisions**: An essential safety concern where parts of the exoskeleton might collide with the user's own body or with other parts of the exoskeleton, potentially causing injury or damage.
- **Human-Exoskeleton Interface**:
  - **Non-invasive Interfaces**: Interfaces that do not penetrate the skin and include straps, vests, and cuffs. These are generally safer and less likely to cause complications.
  - **Invasive Interfaces**: Interfaces that involve surgical implants or other methods that penetrate the skin, offering more direct control at a higher risk of infection and other medical complications.
### Singularity Analysis
- **Mechanical Singularities**: Points in the range of motion of a joint system where the control or behavior of the exoskeleton becomes undefined or highly unpredictable. These can occur if the alignment of rotational axes causes the mechanical degrees of freedom to overlap or align precisely.
- **Example Issues**:
  - **Between Joints 1 and 3**: Around the shoulder internal-external rotation axis in configurations (a) and (b), where typical configurations can lead to limitations in movement or control difficulties.
  - **Between Joints 3 and 5**: Occurs in full elbow extension, where the arm is fully straightened, potentially leading to control instability or mechanical lock-up.
- **Solutions**: Rearranging joints to move mechanical singularities to less used locations of the exoskeleton’s range of motion can mitigate these issues, enhancing both the safety and functionality of the device.

## Hand Exoskeletons
Designing hand exoskeletons poses unique challenges and requirements due to the complex anatomy and functional importance of the human hand. These devices must meet specific criteria to ensure they are effective and comfortable for users. Here’s a breakdown of the requirements and challenges associated with hand exoskeletons:
### Requirements of Hand Exoskeletons
#### 1. **Wearability**
- **Lightweight Structure**: The exoskeleton must be light enough not to burden the user or impede natural movements.
- **Portable Actuation System**: It needs a portable system that can generate the necessary forces to assist or resist during hand movements without requiring fixed, bulky external equipment.
#### 2. **Adaptability**
- **Versatility in Fit**: The device should accommodate various hand sizes, shapes, and ranges of motion (ROM). Research indicates that up to four different biomechanical models are necessary to cover the entire population adequately.
- **Rehabilitation Utility**: The exoskeleton should support multiple rehabilitation protocols to cater to different hand disorders, making it a versatile tool in therapeutic settings.
#### 3. **Comfort**
- **Ergonomic Design**: The mechanism should be lightweight and feature a well-distributed human-exoskeleton interface that avoids discomfort or safety hazards. Proper alignment is crucial to prevent stress on the musculoskeletal structure.
### Challenges of Hand Exoskeletons
![[Pasted image 20240710122334.png#invert|200]]
#### 1. **Alignment and Anatomy**
- The intricate structure of the hand, with its complex joints and limited space, makes it difficult to design exoskeletons that align well without interfering with natural movement.
- Achieving a perfect alignment with the hand’s anatomy, particularly at the joints, is a major technical challenge due to the dynamic nature of hand movements.
#### 2. **Joint-Specific Issues**
- **Finger Joints**: 
  - **DIP and PIP Joints**: These are primarily hinge joints where misalignments are minimal but still crucial for precise movement.
  - **MCP Joints**: These ellipsoidal joints present several difficulties:
    - The distance between MCP and PIP joints changes during flexion.
    - The center of rotation at these joints also shifts during movement.
    - Traditional designs struggle to align actuation systems like pulleys with the joint’s rotational axis due to these changes.
#### 3. **Thumb Challenges**
- **Opposition and Manipulation**: The thumb plays a critical role in grasping and manipulation tasks, making its functionality crucial in hand exoskeleton designs.
- **Design Solutions**: Some portable exoskeletons, such as the HandExos, specifically address thumb opposition to enhance the device's effectiveness in supporting hand functions.
### Hand Structure and Synergies
- **High-Dimensional Configuration Space**: The hand comprises 27 bones, 18 joints, and 21 degrees of freedom (DoFs), managed by 39 intrinsic and extrinsic muscles. This complexity makes direct replication challenging in robotic systems like exoskeletons.
- **Synergies**: These are simplified control strategies that allow complex hand movements to be achieved by coordinating the activation of multiple joints and muscles. Synergies reduce the dimensionality of hand control, making it more feasible to replicate with robotics.
- **Types of Synergies**:
  - **Postural Synergies**: Involve patterns of joint rotations that typically occur together, such as the flexion-extension movements of the DIP (distal interphalangeal) and PIP (proximal interphalangeal) joints.
  - **Muscle Actuation Synergies**: Include coordinated movements like abduction (movement away from the central axis of the body) and adduction (movement toward the body).
### Muscle Synergies
- **Definition**: Muscle synergies refer to the invariant, coordinated activations of groups of muscles that the central nervous system uses to create patterns of muscle activity for movement execution.
- **Observation and Extraction**: These are typically observed through electromyography (EMG) studies, which monitor muscle activations.
- **Types**:
  - **Static Synergies**: Analyze muscle activations during a static grasp using dimensionality reduction techniques like principal component analysis (PCA).
  - **Synchronous Synergies**: Involve time-dependent linear combinations of static synergies.
  - **Time-Variant Synergies**: Describe entire motion patterns as linear combinations of time-dependent synergy trajectories, allowing for dynamic modeling of hand movements.
### Application to Robotics
- **Design Relevance**: Understanding and applying hand synergies in robotic design, such as hand exoskeletons, allows for the development of more natural, efficient, and capable robotic systems.
- **Implementation**:
  - **Exploration Techniques**: These can include analysis based on brain control or muscle activation, manipulation task trajectories, and grasping data. PCA and other statistical methods are often used to identify and extract these synergies.
  - **Optimization**: In robotic hands, synergies can guide the optimization of tendon routing to replicate the complex motions of the human hand more accurately.
### Challenges and Considerations
- **Decodability of Tasks**: While synergies provide a powerful framework for reducing the complexity of hand movements, they do not automatically explain the specific tasks the hand can perform. The relationship between specific synergies and tasks often requires additional analysis.
- **Biomechanical Couplings**: Synergies reveal the biomechanical relationships between muscle activations, offering insights into the structural and functional constraints within which the human hand operates.

## Types of Hand Exoskeletons
1. **Non-wearable**: These are typically stationary setups used for rehabilitation or specialized tasks like surgical simulation. They aren’t designed for continuous personal use or mobility.
2. **End-point based exoskeletons**: These systems interact with the user only at specific points (e.g., fingertips), rather than fitting along the contours of the entire limb.
3. **Kinematically different to the human limb**: These exoskeletons do not mimic the exact joint articulation of the human hand, which can lead to a different feel and range of movement.
4. **Wearable**:
   - **End-point based**: Similar to their non-wearable counterparts but designed for active, personal use.
   - **Kinematically equivalent to the human limb**: These mimic the natural movements and degrees of freedom of the human hand.
   - **Kinematically different**: These may include additional or fewer degrees of freedom than the human hand or arrange them differently to enhance certain types of functionality.
5. **Soft exoskeletons**: Made from flexible materials that conform to the body more naturally, offering comfort and a broader range of unimpeded movement.
### Non-Wearable Hand Exoskeletons:
- **Phantom and Touch Haptic Devices**: Developed by 3D Systems, these are sophisticated tools primarily used in non-mobile settings like labs or design studios. They provide precise haptic feedback, which is crucial for applications requiring detailed manual interaction such as CAD and surgical simulation.
  - **Phantom**: This device is used to simulate interactive environments where precise manipulation of digital objects is necessary. It can track movements in six degrees of freedom and provide force feedback, which helps in creating a realistic sense of touch.
  - **Touch**: Similar to Phantom, but uses a stylus interface. It's especially useful in applications where fine control of a cursor or other digital tools is necessary.
- **Gifu University Hand**: A more complex and robust system, it aims to rehabilitate the human hand by mimicking natural movements through data obtained from a healthy hand wearing a data glove. This system is capable of controlling a significant number of DOFs (18 in total), including individual fingers and the wrist.
  - **Actuation and Sensing**: Powered by servo motors and equipped with rotary motor encoders and three-axis force sensors, this exoskeleton provides detailed feedback and control, essential for effective rehabilitation.
  - **Drawbacks**: The requirement for the user to wear two gloves reduces direct tactile interaction with the environment, which can diminish the effectiveness of therapy or task performance. Additionally, the numerous servomotors required for operation add significant weight, potentially limiting the duration of use and comfort.
### Wearable Hand Exoskeletons
#### End-point based
These exoskeletons are simpler in design because they only need to fixate at the fingertips, avoiding the need to align with each finger joint. They control the movement of the fingers by manipulating the position of the fingertips.
- **Pros**: Easier to control and install; does not require encasing the entire finger structure.
- **Cons**: Limited control over individual finger joints; can only manipulate the overall position and orientation of the fingers.
##### Four-Fingered Light-Weight Exoskeleton:
This specific design is a practical application of the "end-point based" and "kinematically different" exoskeleton concepts. It aims to provide controlled manipulation of the fingers within their natural workspace without encasing the palm or obstructing its movements.
- **Main Features**:
    - **Bidirectional Forces**: Capable of exerting forces in both directions on the finger phalanges, allowing for both pushing and pulling motions.
    - **Adaptability**: Designed to accommodate different hand sizes and the natural workspace of human fingers.
    - **Free Palm Design**: Ensures that the palm remains unobstructed, enhancing comfort and usability.
    - **Support for Four Fingers**: Targets the four main fingers, excluding the thumb, for simplicity and effectiveness.
- **Mechanical Design**:
    - **Direct Driven Mechanism with Bevel Gears**: Provides precise control over finger movement.
    - **Adjustable Linkages**: For palm, proximal, and medial finger phalanges to accommodate various finger lengths.
    - **Active and Passive DoFs**: Active degrees of freedom (DoF) for flexion/extension; passive DoF for abduction/adduction to allow natural lateral movements of the fingers.
- **Specifications and Sensing**:
    - **5W DC Motor**: Powers the movement.
    - **Position and Force Sensors**: Located between key structural elements to measure and control the forces exerted and the position of the exoskeleton.
- **Analysis Tools**:
    - **Fingertip Force Sensors and CyberGlove**: Used in experimental setups to analyze the effectiveness of the exoskeleton in replicating natural finger movements and exerting sufficient force for various tasks.
#### Kinematically equivalent
These mimic the exact joint structure of the human hand, aiming to replicate each finger's natural movements precisely.
- **Pros**: Allows for very natural and intuitive control of each finger.
- **Cons**: More complex in terms of design and alignment of joints; typically heavier and more cumbersome.
##### HANDEXOS
1. **Center of Rotation Alignment**:
   - The exoskeleton's joints are designed to coincide with the human joints, ensuring movements feel natural and intuitive.
   - This precise alignment, however, introduces challenges, such as the need for extra space between the fingers and a high potential for misalignments due to individual anatomical differences.
2. **HANDEXOS Design**:
   - **Optimization Goals**: The design focuses on enhancing kinematic compatibility, wearability, and portability.
   - **Structure**: It features a combination of active and passive Degrees of Freedom (DoFs).
     - **Active DoFs**: These are responsible for the actuation and transmission of power from the robot to the human limb.
     - **Passive DoFs**: These allow the human limb to naturally determine the joint axis positions, accommodating personal anatomical variations.
3. **Joint-Specific Designs**:
   - **PIP and DIP Joints (Proximal and Distal Interphalangeal Joints)**:
     - These joints are aligned directly with the exoskeleton, aiming for a straightforward mechanical correspondence.
     - A soft neoprene cover at the interface helps absorb minor misalignments, enhancing comfort and reducing the risk of pressure points.
   - **MCP Joint (Metacarpophalangeal Joint)**:
     - Features a more complex self-aligning architecture using a parallel chain consisting of one prismatic joint and two revolute joints (PRR configuration). This setup allows for more sophisticated movements and adjustments, decoupling joint rotation from translation, which is crucial for complex finger movements.
4. **Cable-Driven Actuation System**:
   - The use of cables in antagonistic pairs (opposite wrapping directions) allows for bidirectional actuation, providing control over both flexion and extension movements.
   - This system decreases the inertia of moving parts, enhancing response times and reducing the overall strain on the wearer.
   - By locating motors along the arm rather than in the hand or fingers, the system enhances wearability and reduces the load on the fingers, making the exoskeleton more comfortable and less obtrusive.
5. **Underactuation**:
   - Utilizes underactuation to control multiple joints with fewer motors, simplifying the system while covering all necessary motions for the fingers and thumb.
   - This design choice reduces weight and complexity, making the system more practical for everyday use and potentially more affordable.
#### Kinematically different
- These may simplify or complicate the joint structure compared to the human hand, depending on the design goals, such as enhancing strength or dexterity for specific tasks.
    - **Pros**: Can be optimized for specific functionalities or ergonomic improvements.
    - **Cons**: May feel less natural compared to the human hand's movement.
##### Key Concepts and Examples
1. **Remote Center of Rotation Structure**:
    - These exoskeletons do not align the mechanical joint axes directly with human joint axes.
    - This approach can solve alignment problems and enhance adaptability but may introduce complexity and reduce compliance (the ability to adapt or conform to the wearer’s movements).
2. **Non-anthropomorphic, Redundant Structures**:
    - Incorporate additional active or passive joints to allow the structure to self-align with the human limb, which is critical for applications like stroke patient grasp assistance and training.
    - This design can include underactuated finger motion, where multiple fingers are driven by a single motor, often through rigid synergistic coupling, simplifying the design and reducing weight.
#### Soft Exosuits
Made from flexible materials and actuated using mechanisms like cables or soft actuators.
- **Pros**: Highly comfortable and lightweight; good for long-term wear.
- **Cons**: May offer less precise control of finger movements due to the flexible nature of the materials.
1. These systems use flexible or underactuated structures that do not require rigid mechanical alignments, allowing the human hand to provide the skeletal framework for movement.
    - **Example**: NASA and General Motors developed a robotic glove designed to assist astronauts and mechanics by reducing the force exerted during tasks. This glove uses tendon-driven mechanisms and soft pneumatic actuators to enhance grip strength and endurance, reducing the risk of injury from repetitive tasks.
2. **Force Myography for Control**:
    
    - Utilizes sensors to detect changes in muscle size or the pressure exerted by muscles under the skin, helping to control the exoskeleton through natural user intention.
3. **Technological Integration and User Assistance**:
    
    - Examples include exoskeletons that integrate sensors to detect when a tool is grasped, automatically adjusting tendon tensions to maintain grip without continuous user effort.
    - Devices are designed to be lightweight and wearable, often incorporating advanced materials and battery technology to maximize functionality and user comfort.