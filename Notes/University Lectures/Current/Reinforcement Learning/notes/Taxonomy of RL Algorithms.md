## [[Value-Function|Value-based]] 
- such as [[Value-Function Approximation#Deep Q-Learning Network (DQN)|DQN]], [[Value Iteration]], [[Policy Iteration]]
### Details
- In value-based methods, the agent learns a value function, $Q(s, a)$
- The policy $\pi(s)$ is defined implicitly as the action that maximizes the value function for each state $s$. That is, $\pi(s) = \text{argmax}_a Q(s, a)$. This means the policy directly depends on the value function but is not adjusted independently.
- This approach can be either [[Model-Free RL|model-free]], where the agent does not model the environment’s dynamics, or [[Model-based RL|model-based]], where it also learns a model of the environment to predict future states.
### Advantages / Disadvantages
- <mark style="background: #BBFABBA6;">✓</mark> Rather sample-efficient (allows off-policy)
- <mark style="background: #BBFABBA6;">✓</mark> Yield state-of-the-art performance in many domains 
- <mark style="background: #FF5582A6;">×</mark> No convergence guarantees 
- <mark style="background: #FF5582A6;">×</mark> Often quite hard to tune... 
- <mark style="background: #FF5582A6;">×</mark> Hard to use for continuous action spaces 
- <mark style="background: #FF5582A6;">×</mark> Approximation errors in the Q-function might bias the quality of the resulting policy
- **Requires Dynamics Model**: Both value iteration and policy iteration require complete knowledge of the environment’s dynamics through the state transition probabilities P(s′∣s,a)P(s' | s, a)P(s′∣s,a) and rewards R(s,a)R(s, a)R(s,a). This is often impractical or impossible in complex environments.
- **Computationally Intensive**: Iterating over all states and actions can be computationally prohibitive except in small, discrete problems due to the exponential growth of the state-action space with the number of variables in the environment.
### Solutions to Overcome Limitations
Given the limitations of exact methods in handling large or complex state spaces, several approximative techniques have been developed:
#### 1. **Sampling-based Approximations**
- **Description**: These methods use sampling techniques to estimate the value functions and policies without needing to know the entire state transition model. Methods like Monte Carlo simulations randomly sample transitions from the policy and use these samples to estimate the value functions.
- **Advantages**: Reduces the need for a complete model and can handle larger state spaces.
#### 2. **Function Approximation**
- **Description**: Function approximation methods use parametric or non-parametric models to estimate the value functions or policies. Techniques such as neural networks, linear regression, or decision trees can generalize across the state space without needing to compute or store values for every state-action pair.
- **Advantages**: They can efficiently handle high-dimensional and continuous spaces by generalizing from observed samples to unseen states.
## [[Policy Optimization]]
### Details
- Here, the focus is directly on finding the best policy parameters $\theta$ that maximize the expected return $J_\theta$. 
- The parameters $\theta$ of the policy are optimized directly (typically using gradient ascent) to maximize $J_\theta$. This is a more direct method compared to value-based learning as it adjusts the policy parameters based on their performance.
- Like value-based methods, policy search can be either [[Model-Free RL|model-free]] or [[Model-based RL|model-based]].
### Advantages / Disadvantages
- <mark style="background: #BBFABBA6;">✓</mark> Easier to use and tune 
- <mark style="background: #BBFABBA6;">✓</mark> More compatible with rich architectures (including recurrence) 
- <mark style="background: #BBFABBA6;">✓</mark> More versatile, more compatible with auxiliary objectives 
- <mark style="background: #BBFABBA6;">✓</mark> (Almost) no bias -> finds good solutions 
- <mark style="background: #FF5582A6;">×</mark> Needs (much) more samples
## Actor-Critic
### Details
- Actor-Critic methods combine elements of both value-based and policy search methods. The "Critic" estimates the value function $Q(s, a)$, and the "Actor" updates the policy parameters $\theta$ based on the gradient of the expected return guided by the Critic.
- The Actor-Critic architecture allows for simultaneous learning of the policy (actor) and the value function (critic), using the value function to reduce the variance in the policy gradient estimates.
- This approach is typically [[Model-Free RL|model-free]] but can also be extended to include a model of the environment ([[Model-based RL|model-based]]), which helps in learning and planning.

## Why so many methods?
### Different trade-offs:
#### Sample Efficiency
- This refers to how many interactions (samples) with the environment are necessary for an algorithm to learn an effective policy.
#### Stability & Ease of Use
- Some algorithms are easier to implement and more robust to changes in their hyperparameters or variations in the environment.
### Different Assumptions
#### Stochastic or Deterministic
- Some algorithms assume a deterministic environment where the same action in a given state always leads to the same next state, while others handle stochastic environments where actions can lead to different states with certain probabilities.
#### Continuous or Discrete
- This distinction refers to whether the state and action spaces are continuous (e.g., controlling the speed and direction of a car) or discrete (e.g., a game with a finite set of possible moves).
#### Episodic or Infinite Horizon
- Episodic tasks end after a certain sequence of events (like a game ending after a set number of moves), whereas tasks with an infinite horizon continue indefinitely.
### Different Challenges in Different Settings
#### Easier to Represent the Policy
- In some settings, it might be more straightforward to define and optimize a policy directly.
#### Easier to Represent the Model
- In other cases, particularly in model-based RL, it might be more effective or feasible to represent and learn a model of the environment.