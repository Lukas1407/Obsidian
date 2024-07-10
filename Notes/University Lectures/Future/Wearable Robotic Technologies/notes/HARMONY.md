The description you've provided outlines the complex design and testing process for a robotic exoskeleton system called HARMONY, which is equipped with advanced mechanisms to mimic and support the natural movements of the human upper body, particularly the shoulder girdle.
![[Pasted image 20240710111104.png|200]]
### System Components and Functionality

#### **Serial Elastic Actuators (SEA)**
- **Description**: HARMONY utilizes SEAs at all 14 axes of movement. SEAs are essential for safely and accurately controlling the force exerted by the robot, as they allow for more compliant interaction with the human body.
- **Benefits**: This setup enhances the robot's ability to absorb and adapt to dynamic forces during human-robot interaction, which can improve safety and user comfort.

#### **Adjustable Linkages**
- These are used to modify the length of the linkages, allowing the exoskeleton to fit different user sizes and adapt to individual body dimensions dynamically.

#### **Multi-axis Force/Torque Sensors**
- **Placement**: Located at the interaction ports of the wrist and upper arm, these sensors measure the forces and torques exerted during movements.
- **Function**: They are crucial for monitoring and adjusting the robot’s force output to ensure safe and effective assistance.

#### **Support Structures**
- **Wrist Cuff and Handle**: Grounded at the force/torque sensor in the wrist, helping in guiding and supporting the arm movements.
- **Chest Harness**: Attached to the frame, it supports the torso, providing stability and reducing the load on the user's lower back and shoulders.

### Shoulder Girdle Mechanism

#### **Design Considerations**
- **Trajectory Analysis**: Motion capture technology records the movements of the acromion (a part of the shoulder blade) to understand the natural motion paths and the center of rotation shifts during different shoulder movements.
- **Adjustable Mechanism**: Incorporates an adjustable pivot point and a link with an adjustable length to accommodate variations in the shoulder joint's center of rotation during motion.

#### **Configuration**
- **Revolute Joint (J1) and Parallelogram (J2)**: These components shift the circular motions related to shoulder movements, aligning them correctly with the human anatomy.
- **Ball-and-Socket Joint**: Comprises three rotational joints arranged in a serial chain, with intersecting axes to allow a broad range of motion without reaching mechanical singularities.

### Kinematic Design and Testing

#### **Adjustability**
- The lengths of the upper arm and forearm segments are adjustable, ensuring that HARMONY can be used by subjects with varying body dimensions.
- **ROM Testing**: Position sensors track the center of the wrist and the center of rotation of the shoulder joint during free movements, verifying that the exoskeleton can cover the full range of motion needed for daily activities.

#### **Kinematic Compatibility Test**
- **Experimental Setup**: Tests were conducted to measure residual forces and torques during specific shoulder movements, both with the shoulder girdle mechanism free to move and fixed.
- **Results**: Low force values were recorded when the girdle mechanism was free, indicating good kinematic compatibility and confirming that the device supports natural shoulder movements without undue restriction.

### Conclusion
HARMONY’s design effectively addresses the critical aspects of wearable robotic assistance for the upper body. By incorporating adjustable mechanisms, sophisticated joint configurations, and rigorous testing, the system ensures a high degree of safety, comfort, and functional alignment with human biomechanics. This makes HARMONY a promising tool for therapeutic training and augmentation, capable of adjusting to individual user needs and facilitating a wide range of natural movements.