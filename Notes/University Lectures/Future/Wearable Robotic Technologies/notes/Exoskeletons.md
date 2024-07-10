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
   - **Electric**: Powered by electric motors, often used for precise control.
   - **Pneumatic**: Utilizes air pressure for movement, offering lightweight solutions.
   - **Hydraulic**: Uses fluid pressure to achieve high force output, suitable for heavy-duty applications.
### **Power Transmission**:
   - **Gear**: Uses gears to transmit power efficiently.
   - **Cable**: Uses cables, which can make the system lighter and more flexible.
   - **Belt**: Uses belts to transmit power, which can reduce noise and vibration.
### Control Methods:
   - **Impedance**: Modifies mechanical impedance to adapt to changes in the environment.
   - **Force**: Direct force control for robust interactions.
   - **Fuzzy-Neuro**: Advanced algorithms that combine fuzzy logic and neural networks for adaptive control.
### Interface to Human Body:
   - **EMG (Electromyography)**: Interfaces that read electrical activity from muscles to control the device.
   - **FMG (Force Myography)**: Uses pressure sensors over muscles to detect changes in muscle shape.
   - **Mechanical**: Direct mechanical linkage to the human body.
### Application Areas:
   - **Rehabilitation**: Assists with recovery from injury by supporting weakened limbs.
   - **Augmentation**: Enhances the strength or endurance of the wearer.
   - **Substitution**: Replaces lost functionality, such as in cases of limb amputation.
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
- **Example Devices**: NEUROExos, Ekso Bionic Suit, HAL, KIT-EXO.
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
- **Example Devices**: HARMONY, iT-Knee, UtahKnee.
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
- **Example Devices**: Harvard Soft Exosuit, Myosuit.

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