### Ankle Inverse Dynamics
1. **Weight Force of the Foot Segment ($mg$)**:
   - The gravitational force acting downward through the center of mass (CoM) of the foot.
   - **Equation**: 
     $$
     mg = mass \times gravity
     $$

1. **Ground Reaction Force ($F$)**:
   - Acts upward at the center of pressure (CoP) as a reaction to the weight force.
   - Typically equal and opposite to the gravitational force when standing statically.
2. **Ankle Reaction Force ($R_a$)**:
   - Acts at the ankle joint to balance the moments created by other forces.
   - Can be split into horizontal ($R_{ax}$) and vertical ($R_{ay}$) components for analysis.
![[Pasted image 20240709120154.png#invert|250]]
### Ankle Joint Force Calculation

- Using Newton’s Second Law ($F = m \cdot a$), the forces on the ankle can be expressed as:
  - **Horizontal Forces**:
    $$
    F_x + R_{ax} = m \cdot a_x
    $$
  - **Vertical Forces**:
    $$
    F_y + R_{ay} - mg = m \cdot a_y
    $$
  - **Resulting Ankle Joint Forces**:
    $$
    R_{ax} = m \cdot a_x - F_x
    $$
    $$
    R_{ay} = m \cdot a_y - F_y + mg
    $$
  - For a static case ($a_x = a_y = 0$), the equations simplify to:
    $$
    R_{ax} = -F_x, \quad R_{ay} = -F_y + mg
    $$
### Ankle Torque Analysis
1. **Equation for Torques**:
   - The torque at the ankle ($M_a$) is calculated using the moment of inertia ($J$) and angular acceleration ($\alpha$):
     $$
     M = J \cdot \alpha
     $$

2. **Calculating Torque**:
   - **Around the CoM**:
     $$
     M_a = F \cdot l - R_a \cdot r
     $$
     Where $l$ and $r$ are the lever arms of the ground reaction force and ankle reaction force, respectively.

3. **Detailed Torque Components**:
   - Considering components along $x$ and $y$:
     $$
     M_a = F_{y}l_x + F_{x}l_y - R_{ax}r_y - R_{ay}r_x
     $$
   - For static equilibrium ($\alpha = 0$), the net torque is zero, which leads to:
     $$
     M_a = -F_{y}l_x - F_{x}l_y + R_{ay}r_x + R_{ax}r_y
     $$

### Example of Ankle Torque Calculation

- Given values:
  $$
  F_x = 0 \, N, \quad F_y = 700 \, N, \quad m = 1 \, kg, \quad g = 10 \, m/s^2
  $$
  $$
  a_x = 1 \, m/s^2, \quad a_y = 0, \quad \alpha = 0
  $$
  $$
  l_x = l_y = r_x = r_y = 0.05 \, m
  $$
- Calculating the net force and torque:
  $$
  R_x = m \cdot a_x - F_x = 1 \, N
  $$
  $$
  R_y = m \cdot a_y - F_y + mg = -690 \, N
  $$
  $$
  M_a = -F_y \cdot l_x - F_x \cdot l_y + R_y \cdot r_x + R_x \cdot r_y + J \cdot \alpha = -69.45 \, Nm
  $$


