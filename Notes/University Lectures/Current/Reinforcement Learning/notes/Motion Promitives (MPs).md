  Motion primitives are **trajectory generators** that define how an agent moves or performs tasks. A trajectory is represented as:
  $$
  \tau = \{s_0, s_1, \ldots, s_T\} = h(w)
  $$
  - $h(w)$: A function parameterized by $w$, which generates the desired trajectory $\tau$.  
  - $s_t$: The state at time $t$.  

- **How It Works**:  
  - The desired trajectory is generated using parameters $w$, as well as initial conditions such as joint position $y_0$ and velocity $\dot{y}_0$.  
  - This trajectory is then executed using trajectory tracking controllers like Proportional-Derivative (PD) controllers.
### **Properties of Motion Primitives**
1. **Data-Driven**:  
   - Motion primitives can be easily learned from demonstrations, making them highly adaptable to real-world scenarios.
2. **Compact Representation**:  
   - Motion primitives typically require only a small number of parameters (10–100) to represent a trajectory, making them efficient.
3. **Flexible**:  
   - They can be adapted to new situations or environments with minimal changes.
### **Types of Motion Primitives**
- **Dynaic Movement Primitives (DMPs)**:  
  - Focus on encoding trajectories as differential equations that can be modulated dynamically.
- **Probabilistic Movement Primitives (ProMPs)**:  
  - Represent trajectories as probabilistic distributions, which allow for robust trajectory generation under uncertainty.
### **Trajectory Representation**
- **Representation of a Single Trajectory**:  
  The trajectory $y_t$ at time $t$ is expressed as:
  $$
  y_t = \psi_t^T w + \epsilon_y, \quad \epsilon_y \sim \mathcal{N}(0, \sigma^2)
  $$
  - $\psi_t$: A set of **basis functions** that encode the trajectory.  
  - $w$: Weights that determine the contribution of each basis function.  
  - $\epsilon_y$: Gaussian noise to handle variability.  

- **Phase-Dependent Basis Functions**:  
  - $\psi(z_t)$ depends on the phase of the motion, often represented by normalized Gaussian basis functions:
    $$
    \psi_i(z) = \frac{\phi_i(z)}{\sum_{j=1}^K \phi_j(z)}, \quad \phi_i(z) = \exp\left(-\frac{(z - c_i)^2}{h_i}\right)
    $$
    - $c_i$: Center of the basis function.  
    - $h_i$: Width of the basis function.
### **Episodic Reinforcement Learning with Motion Primitives**
- **Objective**:  
  Directly learn the parameters $w$ of the motion primitives to optimize the objective $J(\omega)$:
  $$
  J(\omega) = \int p_\omega(\theta) g(\theta) d\theta, \quad \omega^* = \arg\max_\omega J(\omega)
  $$
  - Here, $\theta = w$.

- **Workflow**:
  1. Policy $\pi_\omega(\theta)$ generates the parameters $w$.  
  2. These parameters generate the desired trajectory $\tau^d = h(\theta)$.  
  3. A controller $f(s_t, s_t^d)$ uses the desired trajectory to compute actions $a_t$.  
  4. The environment executes the actions and returns the next state $s_{t+1}$.
