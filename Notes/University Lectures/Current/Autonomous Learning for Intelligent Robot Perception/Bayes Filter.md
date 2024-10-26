
   - The **Bayes filter** is a general framework for estimating the state of a system over time, based on noisy measurements and uncertain state transitions.
   - It works by iteratively updating the **belief** about the system’s state as new sensor measurements and actions are observed. The belief about the current state is updated using **Bayes' Rule**.
## Bayes Filter Variants:
   - Several well-known filtering techniques are based on the Bayes filter principle:
     - **Kalman Filters**: Used when the system is linear and Gaussian, providing an efficient solution for estimating states.
     - **Particle Filters**: Used for non-linear, non-Gaussian systems by representing the belief with a set of random samples (particles).
     - **Hidden Markov Models (HMMs)**: Used when the state is hidden, and only noisy observations are available.
     - **Dynamic Bayesian Networks (DBNs)**: Graphical models that extend Bayesian networks to dynamic systems.
     - **Partially Observable Markov Decision Processes (POMDPs)**: Used when both the state and the outcome of actions are partially observable, helping decision-making in uncertain environments.

## Bayes Filter Equation:
   - The belief $\text{Bel}(x_t)$ about the current state $x_t$ is updated using the following equation:
     $$
     \text{Bel}(x_t) = \eta p(z_t | x_t) \int p(x_t | u_t, x_{t-1}) \text{Bel}(x_{t-1}) dx_{t-1}
     $$
   - This equation consists of two main steps:
     - **Measurement update**: The term $p(z_t | x_t)$ represents how likely the current measurement $z_t$ is, given the current state $x_t$ (sensor model).
     - **Prediction step**: The integral $\int p(x_t | u_t, x_{t-1}) \text{Bel}(x_{t-1}) dx_{t-1}$ represents how the previous state evolves into the current state, given the action $u_t$ (motion model).
   - The normalizing factor $\eta$ ensures that the belief sums to 1.

