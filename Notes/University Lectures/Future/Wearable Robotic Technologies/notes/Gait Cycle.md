> [!abstract] Definition
>  The gait cycle refers to the period of time from initial contact of one foot to the next initial contact of the same foot

![[Pasted image 20240710083435.png#invert|800]]
- Average walking speed (m/s): 1.37 
- Average step rate (step/s): 1.87 
- Average step length (m): 0.72

- [[Gate Stride]]
- [[Stride Length]]
### Stance and Swing Phases
- **Stance Phase**: This phase accounts for about 60% of the gait cycle, beginning when the foot makes contact with the ground and continuing until the foot begins to lift off. It's critical for weight bearing and propulsion.
- **Swing Phase**: Takes up the remaining 40% of the gait cycle. It begins when the foot lifts off the ground and ends when the foot makes contact with the ground again. This phase allows the leg to move forward and prepare for the next stance phase.
### Toe-off
- This event marks the transition from the stance phase to the swing phase, occurring when the toes leave the ground.
### Single and Double Support
- **Single Support**: Occurs when only one foot is in contact with the ground, which typically happens twice in a single gait cycle (once for each leg).
- **Double Support**: Occurs when both feet are in contact with the ground at the same time. This phase decreases as walking speed increases and is non-existent in running (hence running has a 0% stance phase).
### Running and Double Support
- During running, there is no double support phase; instead, there might even be a phase where neither foot is on the ground, known as the flight phase.
### Cadence
- **Definition**: The number of steps taken per minute. An average walking cadence is typically between 100 to 115 steps per minute.
- **Calculation**: 
  $$
  \text{Cadence in strides per second} = \frac{\text{Cadence in steps per minute}}{120}
  $$
  For example, if the cadence is 80 steps per minute:
  $$
  \text{Cadence in strides per second} = \frac{80}{120} \approx 0.67 \, \text{strides per second}
  $$
### Speed Calculation
- **Formula**: Speed can be calculated by the product of cadence and stride length, divided by 120 to convert the rate to the appropriate unit.
  $$
  \text{Speed} = \frac{\text{Cadence} \times \text{Stride Length}}{120}
  $$
### Stride Time
- **Definition**: The time it takes to complete a full stride.
- **Calculation**: 
  $$
  \text{Stride Time} = \frac{120}{\text{Cadence}}
  $$
  This gives the stride time in seconds per stride, based on the number of strides per minute.

![[Pasted image 20240710083841.png#invert|600]]
## Events
- The key events of the gait cycle define the ![[Temporal Spatial Parameters (TSP)]]
## Motion Capture Systems and Sensors in Gait Analysis:
1. **Motion Capture Systems**:
   - These systems track the motion of body segments to calculate joint angles, observe limb trajectories, and estimate the center of mass. This data is crucial for creating detailed analyses of movement patterns in real-time.
2. **Force Plates**:
   - Integrated into walkways or treadmills, these devices measure the forces exerted by the feet during walking, providing data on load distribution and balance.
3. **Inertial Measurement Units (IMUs)**:
   - These sensors measure linear acceleration and orientation, offering portable options for gait analysis outside the laboratory setting.
4. **Force Sensors**:
   - Measure interaction forces between the human body and devices like orthotics or prosthetics. This is essential for ensuring that such devices are properly tuned to the user’s needs.
5. **[[Electromyography (EMG)]]**:
   - Measures muscle activity and can be used to infer force production during different phases of the gait cycle. This helps in understanding muscle coordination and timing during walking.
## Kinematics and Torque
### Kinematics in Frontal Plane During Walking
- the plane that divides the body into front and back
#### Kinematics of the Knee Joint 
![[Pasted image 20240710084735.png#invert|400]]
- **Abduction and Adduction**: The graph shows the knee joint's motion in terms of abduction (movement away from the midline of the body) and adduction (movement toward the midline). These movements are measured in degrees throughout the gait cycle.
- **Individual vs. Average Data**: The black lines represent individual data points from subjects, showing variability in knee movement among them. The red line represents the average movement across five subjects, highlighting the typical pattern of knee motion during walking.
- **Observations**:
  - During the stance phase of the gait cycle (approximately 0-60% of the cycle), there is generally a transition from slight adduction to abduction.
  - The maximum abduction generally occurs around 80% of the gait cycle, aligning with the late stance to early swing phase transition.
  - This abduction helps in stabilizing the knee as the foot prepares to leave the ground and the body shifts weight to the opposite leg.
#### Kinematics of the Hip Joint
![[Pasted image 20240710084756.png#invert|400]]
- **Hip Movement**: This graph tracks the hip's abduction and adduction movements during the gait cycle.
- **Stance Phase**:
  - Begins with hip adduction, which is the movement of the leg towards the body’s midline, aiding in supporting the body’s weight over the standing leg.
  - The graph shows that as the stance phase progresses, there's a reversal towards abduction, particularly noticeable towards the end of the stance phase.
- **Swing Phase**:
  - Characterized by a clear trend toward hip abduction, facilitating the leg's forward movement and preparing for the subsequent foot strike.
### Ankle Movement Considerations
- The discussion notes that ankle rotation in the horizontal plane is very slight and generally not considered significant in this context. This indicates that while there is some rotation at the ankle, its impact on walking kinematics in the frontal plane is minimal.
### Kinematics in Sagittal Plane During Walking
#### Ankle Joint Motion:
![[Pasted image 20240710085025.png#invert|400]]
   - **Dorsiflexion and Plantarflexion**: Shows the angles of dorsiflexion (foot moves upward) and plantarflexion (foot points downward) throughout the gait cycle.
   - **Key Phases**:
     - Initial plantarflexion as the foot prepares for ground contact, transitioning to dorsiflexion to accommodate the terrain and absorb impact during the mid-stance.
     - Increased plantarflexion towards the end of the stance phase to push off the ground, leading into the swing phase.
#### Hip Joint Motion:
![[Pasted image 20240710085048.png#invert|400]]
   - **Flexion and Extension**: Shows the hip moving from flexion (thigh moves forward) to extension (thigh moves backward) and back to flexion throughout the cycle.
   - **Key Phases**:
     - Begins with hip extension during stance as the body moves over the supporting leg.
     - Transitions to hip flexion during swing to advance the leg for the next step.
#### Knee Joint Motion:
![[Pasted image 20240710085110.png#invert|400]]
   - **Flexion and Extension**: Tracks the bending (flexion) and straightening (extension) of the knee.
   - **Key Phases**:
     - Slight flexion in the early stance to absorb impact, followed by a relatively stable phase with minimal flexion.
     - Larger flexion during swing to clear the ground.
### Torques in the Sagittal Plane During Walking
#### Ankle Torque:
![[Pasted image 20240710085142.png#invert|400]]
   - Reflects the plantarflexion and dorsiflexion torques normalized to body weight.
   - **Key Observation**: High plantarflexion torque (up to 2x body weight) at the end of the stance phase, facilitating the push-off necessary for propulsion.
#### Hip Torque:
![[Pasted image 20240710085215.png#invert|400]]
   - Shows periodic patterns of flexion and extension torques, correlating with the motion of swinging the leg forward and pulling it back.
   - **Key Observation**: Torques alternate between positive and negative values, indicating changing demands on the hip muscles to drive and control leg movement.
#### Knee Torque:
![[Pasted image 20240710085241.png#invert|400]]
   - Indicates the forces acting to bend and straighten the knee, with values also normalized to body weight.
   - **Key Observation**: Generally negative torque during single-legged support phases, highlighting the knee's role in supporting body weight and stabilizing the leg.
## [[Ground Reaction Forces (GRF)]] During Walking
### Single vs. Double Support
![[Pasted image 20240710085624.png#invert|600]]
- **Single Support**: Occurs when only one foot is in contact with the ground. Higher vertical GRFs are observed as the body's full weight is supported on one leg.
- **Double Support**: Occurs when both feet are briefly in contact with the ground. The vertical GRF is distributed between both legs, showing a relative dip in the force exerted by each foot.
## Inverted Pendulum Model
- Walking can be modeled as an inverted pendulum, where the CoM arcs over the base of support formed by the feet. This model simplifies the complex dynamics of human walking into a more manageable form for analysis.
![[Pasted image 20240710085803.png#invert|400]]
- [[Center of Mass (CoM)|CoM]] Movement:
    - **Falls During Double Support**: As the body transitions between steps, the CoM falls lower, which helps in transferring weight from one leg to another without requiring excessive muscular effort.
    - **Rises During Single Support**: The CoM rises as the swinging leg moves forward, leveraging momentum to help lift the body, making the step more efficient and reducing energy expenditure.