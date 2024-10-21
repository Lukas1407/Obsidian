## Experiment
- **Experiment Setup**: Human subjects performed grasp motions for various objects, imagining grasping 57 different objects while hand movement was captured using a dataglove.
- **Data Analysis**: Principal Component Analysis (PCA) was applied to the data to identify major patterns.
- **Results**: The analysis found that hand movements during grasping occupy a low-dimensional subspace. With the first two principal components, 80% of the variance was captured, and with three components, up to 97%.
- **Implications**: This suggests that complex hand postures during grasping can be effectively described with a few key patterns or synergies, which simplifies the control needed in prosthetic or robotic hand design.

### Postural Synergies and Eigengrasps
- **Principal Component Analysis (PCA)**: This statistical tool is used to analyze large sets of grasping data to extract the principal components (Eigengrasps) that represent the primary modes of variation within the data. Each grasp can be considered as a point in a high-dimensional space (ℝ^d), where d is the number of degrees of freedom (DoFs) in the hand.
- **Eigengrasps**: These are vectors that approximately couple joint movements together, simplifying the control needed by reducing the dimensionality of the motion space. The concept of Eigengrasps or synergies is that many complex hand movements can be approximated by a linear combination of a few base movements (principal components).
- **Dimensionality Reduction**: By using PCA, the basis of Eigengrasps $b$ (where $b$ is much less than $d$) can represent most grasps within a reduced $ℝ^b$ space. This makes it possible to replicate a wide range of grasps using fewer control inputs.
![[Pasted image 20240715120125.png#invert|800]]
### Underactuation in Robotic Hands
- **Mechanical Implementation**: The concept of underactuation in robotic hands is inspired by the biological configuration of vertebrate limbs, where muscles pull on tendons to create movement. In robotics, this is mimicked using a tendon-pulley system.
- **Single Motor Multi-Joint Control**: Underactuation involves using a single motor to control multiple joints. This is achieved through a transmission mechanism, often involving cables and pulleys.
- **Tendon-Pulley System**: As illustrated in the image, the tendons connect to pulleys at various points along the robotic hand or arm. The force $F$ applied by the motor is transmitted through these tendons, generating torques at the joints depending on the radius of the pulley at each joint and the tension in the tendon.
  - $\tau_1 = F \times R_1$
  - $\tau_2 = F \times (P_2 - C_2)$ — where $P_2 - C_2$ effectively represents the lever arm at the second joint, which translates to the radius $R_2$ in a simplified scenario.

### Synergistic Control via Tendon-Pulley System
- **Synergy Implementation**: By pulling on one tendon with one motor, multiple joints can be moved in a coordinated fashion, implementing a synergy. This allows for complex joint movements using simpler and fewer control inputs.
- **Efficiency and Simplicity**: This system reduces the complexity and weight of the robotic hand by minimizing the number of motors and mechanical parts needed, while also closely mimicking the natural movement patterns seen in the human hand.

This underactuated, synergy-based approach provides an efficient way to control robotic hands with high degrees of freedom, making them capable of performing complex tasks with minimalistic control schemes. It's particularly valuable in areas like prosthetics and humanoid robotics, where mimicking human hand dexterity and strength with limited hardware is crucial.

### Implementation of Underactuation
- **Challenges in Exoskeleton Design**: Designing hand exoskeletons that effectively mimic the 21 degrees of freedom (DOF) of the human hand is challenging due to space and complexity constraints.
- **Underactuation**: This refers to the design approach where a single actuator controls multiple joints. This method simplifies the mechanical and control systems by reducing the number of actuators needed.
- **Synergetic Couplings**: In human hands, natural synergies reduce the need for independent control of each joint. Implementing these synergetic couplings through underactuated mechanisms in robotic hands or exoskeletons can offer several benefits:
  - **Space Efficiency**: By reducing the number of actuators, the design becomes more compact, which is crucial for wearable technology.
  - **Reduced Complexity**: Fewer actuators simplify the control architecture, making the system easier to manage and potentially more robust.
  - **Enhanced Functionality**: By leveraging natural hand synergies, these systems can achieve more intuitive and effective grasping and manipulation capabilities.