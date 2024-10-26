![[Pasted image 20241025142535.png#invert|400]]
   - In the diagram, the variables $x_t$ represent the system’s state at time $t$, and the variables $z_t$ represent the sensor measurements at time $t$. The circles for $u_t$ represent the actions or control inputs taken at each time step.
   - Arrows between states indicate **state transitions** (how the state evolves over time), while arrows from states to measurements represent how the current state affects sensor readings.
   - The DBN models how states change over time and how observations are made based on the current state.
## Markov Assumptions
   - The DBN incorporates two important **Markov assumptions**:
     - **Measurement**: The sensor measurement $z_t$ at time $t$ depends only on the current state $x_t$ (not on any previous states or actions). This is the **sensor model**:
       $$
       p(z_t | x_0:t, u_1:t, z_1:t-1) = p(z_t | x_t)
       $$
     - **State Transition**: The system’s state $x_t$ at time $t$ depends only on the previous state $x_{t-1}$ and the action $u_t$ (not on earlier states or measurements). This is the **motion model**:
       $$
       p(x_t | x_0:t-1, u_1:t, z_1:t) = p(x_t | x_{t-1}, u_t)
       $$
   - These assumptions allow the system to be modeled efficiently, because we only need the current state and the immediate previous state, which is a key feature of Markov models.


