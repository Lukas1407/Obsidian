> [!summary] Definition
> TD Learning is a blend of [[Monte-Carlo Estimation]] methods and Dynamic Programming (DP). It aims to estimate the value function—how good it is to be in a given state—by using sample transitions rather than full knowledge of the environment’s dynamics. 

- **Intuition**: The intuition behind TD Learning is that it updates estimates based on the difference (error) between successive predictions. This allows for learning to occur after each step rather than waiting until the end of an episode, as in MC methods.
- **TD Error**: The core of TD Learning is the TD error, which is the difference between the estimated value of the current state and the estimated value of the next state, adjusted by the reward received and the discount factor.

## Equation
$$V^{\pi}(s_t) \leftarrow V^{\pi}(s_t) + \alpha [r(s_{t},a_{t}) + \gamma V^{\pi}(s_{t+1}) - V^{\pi}(s_t)]
,$$where the <mark style="background: #FFB86CA6;">targets</mark> are: $y_{t}=r(s_{t},a_{t})+\gamma V^{\pi}(s_{t+1})$
and the <mark style="background: #FFB86CA6;">temporal difference</mark> (error): $\delta_{t}=r(s_{t},a_{t}) + \gamma V^{\pi}(s_{t+1}) - V^{\pi}(s_t)$:
- It’s essentially a measure of the prediction error in the value function.
- **Learning Signal**: This error serves as a learning signal. If $\delta_t$ is positive, it means the actual outcome was better than expected, and the value of the current state should be increased. Conversely, if $\delta_t$ is negative, the outcome was worse than expected, and the value should be decreased.
- **Updating Estimates**: The TD error is used to update the value function, making it more accurate over time. The learning process involves adjusting the value estimates in the direction that reduces the error.
- **Balance Between States**: It balances the immediate reward and the future value. The term $\gamma V^{\pi}(s_{t+1})$ accounts for the future value of the next state, while $V^{\pi}(s_t)$ is the current estimate. The difference is adjusted by the immediate reward $r(s_t, a_t)$.
- **Intuition**: Think of it like adjusting your expectations based on new experiences.

## Algorithm
![[Pasted image 20240313084146.png#invert|600]]
- However, the [[Value-Function#State-Value Function/V-Function|V-Function]] alone doesn’t tell us which action to take; it only evaluates the state. Therefore, it cannot be used directly for policy improvement, which requires knowing the value of taking specific actions.
	- We don’t have a model of the environment, meaning we don’t know the transition probabilities or the rewards for each state-action pair beforehand.
	- This is where the V-function falls short for policy improvement because it doesn’t provide action-specific guidance without a model.
- Since the [[Value-Function#Action-Value Function/Q-Function|Q-Function]] evaluates both states and actions, it can be updated using TD Learning without needing a model of the environment, making it suitable for policy improvement.
	- By updating the Q-function with TD Learning, an agent can learn which actions are better in each state based on the rewards and future values, leading to improved policies over time.
	- -> [[Temporal Difference Learning#Sample-based Q-Value Iteration|Sample-based Q-Value Iteration]]
- Only works for discrete states

## Sample-based Q-Value Iteration
- A form of Temporal Difference (TD) learning applied to Q-values
$$Q^{\pi}(s_t,a_t) \leftarrow Q^{\pi}(s_t,a_t) + \alpha [r(s_t,a_t) + \gamma \max_{a'} Q(s_{t+1},a') - Q(s_t,a_t)]
,$$ where the targets are $y_{t}=r(s_{t},a_{t})+\gamma \max_{a'}Q^{\pi}(s_{t+1},a')$
and the temporal difference (error) $\delta_{t}=r(s_{t},a_{t})+\gamma  \max_{a'}Q^{\pi}(s_{t+1},a')-Q^{\pi}(s_{t},a_{t})$
- Allows learning the Q-function from experiences without needing a complete mode
### Disadvantages
- No exploration
- By using the greedy policy in [[Temporal Difference Learning#Sample-based Q-Value Iteration|Sample-based Q-Value Iteration]], we might miss out better actions where the value is still wrongly estimated
- Solution [[Temporal Difference Learning#Q-Learning|Q-Learning]]

## Q-Learning
![[Pasted image 20240313085853.png#invert|600]]
- A type of TD-learning because it updates the Q-values based on the TD error
- Is widely used because it’s simple, effective, and can find the optimal policy even when the policy being followed isn’t optimal
- Is considered [[Off Policy]] because it <mark style="background: #FFB86CA6;">updates its Q-values using the Q-value of the next state that has the maximum value</mark> (according to the current estimate), regardless of the policy being followed to generate the data. This means it can learn from transitions that are outside the current policy, including past experiences or hypothetical scenarios.
	- Since Q-learning is off-policy, <mark style="background: #FFB86CA6;">it doesn’t learn the Q-function of the policy generating the transitions</mark>. Instead, <mark style="background: #FFB86CA6;">it learns the Q-function of the optimal policy</mark>, which is the best possible policy that maximizes rewards over time.
- The use of the max operator in the Q-learning update rule allows it to learn the optimal Q-function. This is because it always considers the best possible future action, even if the policy being followed doesn’t select this action.
### Caveats
- Ideally, <mark style="background: #FFB86CA6;">every state-action pair should be visited an infinite number of times</mark> to ensure that the Q-values converge to the optimal values. This is often not feasible in practice due to the potentially vast size of the state-action space.
	- The assumption is that all states and actions are visited infinitely often, which, in theory, means that the policy used to select actions becomes irrelevant in the limit. This is because the Q-values would eventually converge to the optimal values regardless of the policy.
- The learning rate $\alpha$ determines how much new information affects the existing Q-value estimate. It <mark style="background: #FFB86CA6;">needs to be reduced over time to ensure convergence</mark>, but if it’s decreased too quickly, the Q-values may not converge to the optimal values because there hasn’t been enough exploration.
	- The learning rate $\alpha$ should satisfy the condition:$$\sum_{t=0}^{\infty}\alpha_{t}(s,a)=\infty \ \text{and} \ \sum_{t=0}^{\infty}\alpha_{t}(s,a)^{2}<\infty$$
	- This means that the learning rate should not decrease too quickly; it should allow infinite accumulation over time but with a finite sum of squares to ensure convergence.
	- For example $\alpha_{t}=\frac{1}{t}$
	- In practice, a constant learning rate is often used because it’s simpler and can still lead to good results, even if it doesn’t guarantee convergence to the optimal Q-values.
### Overestimation
The max operator in the target value formulation is the primary source of overestimation bias in Q-learning. Here’s why:
- **Statistical Overestimation**: If you consider two random variables $X_1$ and $X_2$, the expectation of their maximum, $\mathbb{E}[\max(X_1, X_2)]$, is always greater than or equal to the maximum of their expectations, $\max(\mathbb{E}[X_1], \mathbb{E}[X_2])$. This is due to the upward bias introduced by taking the maximum of random variables, which tends to overestimate because it picks the highest outcomes.
- **In Q-Learning**: When calculating $\max_{a'} Q(s_{t+1}, a')$, you’re selecting the highest Q-value among potentially noisy estimates. These Q-values are themselves estimates that may include errors. Selecting the maximum value systematically biases the estimates upward, as it tends to pick the most optimistic (overestimated) values.
#### Why This Matters
- **Overestimation Propagation**: This bias can propagate through updates, <mark style="background: #FFB86CA6;">leading to systematically inflated Q-values</mark>. As the Q-learning algorithm progresses, <mark style="background: #FFB86CA6;">these errors can accumulate, resulting in a policy based on over-optimistic expectations</mark> of achievable rewards.
- **Impact on Learning and Convergence**: The overestimation can affect the algorithm’s ability to learn the optimal policy and converge to the true Q-values. It might cause the agent to prefer actions that seem beneficial based on inflated Q-values but are suboptimal in reality.
#### Addressing Overestimation
Several methods have been proposed to mitigate overestimation bias in Q-learning:
- **[[Double Q-Learning]]**: This approach uses two Q-value estimators. Each update for a Q-value uses one estimator for selecting the best action and another for evaluating the action. By decoupling action selection from value estimation, it helps to reduce overestimation.
- **Clipped Double Q-Learning**: Introduced in algorithms like Twin Delayed DDPG (TD3), this method <mark style="background: #FFB86CA6;">clips the Q-value used for the target by taking the minimum Q-value predicted by two separate critics</mark>, further reducing the risk of overestimation.
- **Regularization and Smoothing**: Techniques that regularize or smooth the Q-value updates can also help mitigate the impact of noisy or overestimated values.
## Advantages
- Less noisy and more sample-efficient compared to [[Monte-Carlo Estimation]]
- It incrementally adjusts value estimates towards more accurate predictions with each observed transition.
## Challenges
Basic Q-learning maintains a table of Q-values for every state-action pair. However, in many realistic scenarios, the state space can be so large that it’s impractical or impossible to:
- Visit every state-action pair enough times to learn accurate Q-values.
- Store all the Q-values in memory due to the sheer number of possible states.
### Solution
Generalization through [[Value-Function Approximation]]