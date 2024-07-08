![[Pasted image 20240312135847.png#invert|600]]
  
Once the optimal value function $V^{*}(s)$ is found, the optimal policy $\pi^{*}(s)$ can be derived by choosing the action that maximizes the expected return from each state:
$$
π^∗(s)=\operatorname*{argmax} \limits_{a}\sum_{s}P(s'∣s,a)[r(s,a)+γV(s')]$$
### Using the Q-Value Function
- use the following in the update step:$$Q^{*}_{k}(s,a)=\left(r(s,a)+\gamma\sum_{s'}p(s_|s,a)\max_{a'}Q^{*}_{k-1}(s',a')\right)$$

## Why It Works
The algorithm works because of the **Markov Property**, which states that the future is independent of the past given the present. This allows the algorithm to break down the decision process into simple, manageable steps without losing the essence of the overall problem.

Value iteration is an example of **Dynamic Programming** and is guaranteed to converge to the optimal values as it effectively navigates the trade-off between exploration (trying out new actions) and exploitation (choosing the best-known action).
The content you provided discusses some limitations and complexities of applying traditional reinforcement learning (RL) techniques like value iteration in real-world scenarios. Here’s a breakdown of the main points and their implications:
### Limited Applicability of Traditional Methods
- **Discrete Systems**: Traditional RL methods are effective in environments where states and actions can be distinctly enumerated (i.e., discrete systems). However, many real-world environments have continuous states or actions, where the possible values can vary along a continuum, making them more complex to handle.
- **Linear Systems, Quadratic Reward, Gaussian Noise (LQR)**: These methods also work well in systems that can be described linearly with quadratic rewards and Gaussian noise, a setup common in control theory known as Linear Quadratic Regulator (LQR) problems. However, most real-world systems exhibit non-linear behaviors, which are not adequately addressed by these simple models.
### Necessity for Approximations
Given these limitations, the application of RL in more complex or realistic settings requires approximations:
- **Representation of the V-Function**: In continuous state spaces, representing the value function $V$ becomes challenging. The V-function, which in discrete cases can be represented simply as a table or matrix, needs a form of functional approximation in continuous domains, such as using neural networks, polynomial functions, or other techniques that can generalize across continuous values.
- **Solving for Optimal Actions**: The optimization problem $\max_{a} Q^*(s, a)$ becomes more difficult in continuous action spaces because it's not feasible to evaluate $Q^*(s, a)$ for every possible action $a$ due to the infinite possibilities. This requires methods like gradient ascent or other optimization techniques to find the best action.
- **Expectation Calculation**: The calculation of expected values, such as $\mathbb{E}[V^*(s') | s, a]$, which involves averaging over possible next states $s'$ weighted by their transition probabilities, is complicated when the model and functions involved are arbitrary. This often necessitates numerical integration, sampling methods, or other stochastic estimation techniques.
### Practical Implications
- **Approximate Dynamic Programming**: In real-world applications, techniques like Approximate Dynamic Programming (ADP) or methods leveraging function approximators (e.g., deep learning) are used to address the complexities introduced by high-dimensional, continuous, and non-linear environments.
- **Algorithm Choice**: The choice of algorithm might shift towards more complex solutions like Deep Q-Networks (DQN) for discrete actions in high-dimensional spaces, or Actor-Critic methods where both policy (actor) and value function (critic) are approximated and learned simultaneously.
- **Computational Complexity**: Handling continuous, non-linear environments significantly increases the computational complexity and resources required, influencing how algorithms are implemented and scaled.
### Conclusion
The traditional, straightforward RL methods have proven to be powerful tools for a range of problems but are often too limited for direct application in complex, real-world systems without modifications or approximations. Modern RL research and applications focus on developing and utilizing advanced methods that can handle the complexity and variability of real-world environments effectively.




## Comparison to [[Policy Iteration]]
- **Operational Differences**: In Policy Iteration, each cycle involves fully evaluating a policy before attempting to improve it, which can make it slower per iteration compared to Value Iteration. In contrast, Value Iteration continuously updates the state values and implicitly the policy in a single step, which can lead to faster convergence in terms of the number of iterations.
- **Policy Tracking**: Value Iteration does not explicitly track or update a policy until convergence. Instead, it focuses on the values, and the optimal policy is derived by choosing the best actions given the final values. Policy Iteration explicitly updates and improves the policy at each step.
- **Efficiency**: Policy Iteration can be more computationally intensive per iteration because each policy evaluation step requires solving either a set of linear equations or running enough iterations to converge the values under the current policy. However, it often requires fewer iterations to converge.
- **Implementation Simplicity**: Value Iteration is generally simpler to implement because it only requires a consistent update formula without the need to alternate between evaluation and improvement phases.

## Complexity 
$O(S^2*A)$ 

## Example
![[VI_example-ezgif.com-speed.gif#invert|800]]