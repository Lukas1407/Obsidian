> [!summary] 
> Instead of keeping a table of all values, we use a parameterized [[Value-Function]].
> 
- **Generalization**: Instead of trying to learn and store Q-values for every possible state-action pair, VFA aims to learn about a smaller number of states from experience and <mark style="background: #FFB86CA6;">generalize this knowledge to new, similar states</mark>.

- **Representation**: VFA represents the [[Value-Function#State-Value Function/V-Function|V-Function]] or [[Value-Function#Action-Value Function/Q-Function|Q-Function]] with a parameterized function $Q_{\theta}(s, a)$, where $\theta$ represents the parameters of the function.
- **Learning**: The parameters $\theta$ are adjusted during the learning process to minimize the difference between the predicted Q-values and the target Q-values (which are based on the observed rewards and the estimated values of subsequent states).
## Advantages 
- Allows handling environments with <mark style="background: #FFB86CA6;">large or continuous state spaces</mark> by providing a compact representation that can generalize across states and actions.
- Estimate the value of states and actions, rather than relying on a potentially unmanageable table of Q-values
## Fitting a Value Function
“Fitting” in the context of value functions refers to the process of adjusting the parameters of a model (value function) so that it best represents the relationship between states, actions, and rewards based on the data (experiences) collected from the environment.
### 1. Fit Monte-Carlo Returns
Monte Carlo (MC) methods involve using complete episodes of interaction with the environment to directly calculate the returns (total accumulated reward) for each state visited. The value of a state is estimated by averaging the returns from that state across multiple episodes, considering only the first visit to the state within each episode for a pure "first-visit MC" approach, or all visits in an "every-visit MC" approach.
**Steps:**
- **Collect Data:** Play out full episodes while following a certain policy, recording the states visited and rewards obtained.
- **Calculate Returns:** For each state encountered in an episode, calculate the total return from that state to the end of the episode.
- **Regression on Returns:** Fit the value function $V(s)$ by regressing these observed returns against the states. This can be done using linear regression, neural networks, or any other regression model.$$L=\sum_{t} (V_{\theta}(s_{t})-R_{t})^{2}$$
**Pros and Cons from Image:**
- **Pros:**
  - Can use any supervised learning method.
  - Yields an unbiased estimate of the value function.
  - Always converges (as it's essentially a regression problem).
- **Cons:**
  - High variance in returns, necessitating lots of data to achieve reliable estimates.
  - Can't use the last few transitions of an episode as training data if episodes end due to time limits or other administrative censors.
### 2. Q-Learning with Neural Networks
This approach uses Q-learning, an off-policy TD (temporal difference) learning algorithm, enhanced with the function approximation capabilities of neural networks, often referred to as Deep Q-Networks (DQN).
**Steps:**
- **Action-Value Updates:** For each transition (s, a, r, s'), update the Q-values based on the formula $Q(s, a) \leftarrow Q(s, a) + \alpha (r + \gamma \max_{a'} Q(s', a') - Q(s, a))$.
- **Neural Network as Function Approximator:** Instead of maintaining a table of Q-values for each state-action pair, a neural network is trained to approximate these Q-values. The inputs to the network are states, and the outputs are Q-values for each possible action.
**Pros:**
- Powerful in high-dimensional spaces due to the generalization capabilities of neural networks.
- Does not require the model of the environment, as it learns from observed transitions.
**Cons:**
- Can be unstable or diverge if not carefully implemented due to the inherent approximations and the use of the max operator in noisy environments.
### 3. Fitted Q-Iteration
Fitted Q-Iteration is a batch-based reinforcement learning algorithm where a regression model is used to fit the Q-function iteratively.
**Steps:**
- **Batch Updates:** Instead of updating the Q-values online (after each transition), this method involves collecting a batch of transitions and then performing an update for all these transitions in one go.
- **Regression on Q-Values:** Use a regression model to approximate the Q-function. For each transition in the batch, calculate the target Q-value as $r + \gamma \max_{a'} Q(s', a')$, and then update the regression model to minimize the difference between predicted and target Q-values.
**Pros:**
- Stable and effective in environments where collecting data is expensive.
- Suitable for offline learning, using previously collected data.
**Cons:**
- Requires a large and representative dataset of transitions.
- The convergence depends heavily on the quality of the regression model and the representativeness of the data used.
## Approximate Q-Learning
- Updating the Q-function with gradient descent
- To minimize the loss $L_t = \left(Q_\theta(s_t, a_t) - y_t\right)^2$, we use gradient descent. The parameters $\theta$ are updated by moving in the direction that most reduces the loss. This is done by computing the gradient of $L_t$ with respect to $\theta$ and updating $\theta$ in the opposite direction of this gradient:
$$ \theta_{\text{new}} = \theta - \alpha \frac{d}{d\theta} L_t $$where $\alpha$ is the learning rate, a hyperparameter that controls how much the parameters change in response to the calculated error.
- Expanding the gradient computation gives:

$$ \frac{d}{d\theta} L_t = \frac{d}{d\theta} \left(Q_\theta(s_t, a_t) - y_t\right)^2 = 2(Q_\theta(s_t, a_t) - y_t) \frac{d}{d\theta} Q_\theta(s_t, a_t) $$
- Thus, the update equation becomes:
$$ \theta_{\text{new}} = \theta - 2\alpha (Q_\theta(s_t, a_t) - y_t) \frac{d}{d\theta} Q_\theta(s_t, a_t) $$


### Parameter Update with [[Temporal Difference Learning|TD Error]]
- The parameters are updated by moving in the direction of the gradient of the Q-function scaled by the TD error. 
- This ensures that the parameters are adjusted to reduce the prediction error.

$$\begin{align*}\theta_{\text{new}} &= \theta + \alpha(r(s'_t,a'_t) + \gamma \max_{a'}Q(\theta(s'_t+1,a')) - Q(\theta(s'_t,a'_t))) \frac{d}{d\theta}Q(\theta(s'_t,a'_t))\\
&= \theta + \alpha\delta_{t} \frac{d}{d\theta}Q_{\theta}(s'_t,a'_t)
\end{align*}$$
- $\frac{d}{d\theta}Q_{\theta}(s_t,a_t)$: The gradient of the Q-function with respect to its parameters. This tells us in which direction to change the parameters to increase the Q-value for the state-action pair $(s_t, a_t)$
- In approximate Q-learning, the gradient of the Q-function is multiplied with the TD error to update the parameter vector, which allows the model to learn from the experiences and improve the policy iteratively
![[Pasted image 20240313093558.png#invert|600]]
### Problem
- **No Clearly Defined Objective**: The <mark style="background: #FFB86CA6;">objective isn’t fixed</mark> because the targets (the optimal future values) are not known in advance and change as the agent learns.
- **Changing Targets**: The target values $y_{t}=r(s_{t},a_{t})+\gamma \max_{a'}Q^{\pi}(s_{t+1},a')$  in Q-learning are <mark style="background: #FFB86CA6;">based on the current estimate of the future rewards</mark>. As the agent learns and updates its policy, these estimates change. This means that the ‘ground truth’ the agent is trying to learn from is a moving target, which complicates the learning process.
- **Similarity of Successive States**: In many environments, successive states $s$ and $s'$ are very similar to each other. This similarity means that when the agent updates its estimate for one state, it likely affects its estimates for similar states. This can lead to instability in the learning process because changing the estimate for one state changes the targets for many others.
- **No Gradient Computed**: Because the targets are constantly changing, <mark style="background: #FFB86CA6;">you can’t compute a stable gradient to optimize</mark> as you would in other types of machine learning. The gradient is the direction in which you should adjust your parameters to improve your predictions, but if the ‘correct answers’ keep changing, it’s hard to know which way to go.
### Solutions 1: Use the real gradient
-  Instead of using approximate gradients, this method uses the true gradient of the Q-function with respect to its parameters. This <mark style="background: #FFB86CA6;">leads to a more accurate update rule</mark>, which is expected to result in better learning performance. $$\theta_{\text{new}} = \theta - \alpha \frac{d}{d\theta} L_t = \theta - \alpha \frac{d}{d\theta} (Q_\theta(s_t, a_t) - y_t)^2
$$
- This minimizes the Mean Squared TD Error (MSTD), which <mark style="background: #FFB86CA6;">is a biased objective because it doesn’t lead to Bellman optimality</mark>. It’s calculated as the expectation of the squared difference between the sum of rewards and discounted future state-action values, and current state-action values.
$$\begin{align*}\\\\
\text{MSTD} &= \mathbb{E}_{\pi}[(r_{t} + \gamma \max_{a'}Q_\theta(s', a') - Q_\theta(s, a))^{2}\\
&= \int\mu^{\pi}(s)\int\pi(a|s)\int p(s'|s,a)(r_{t}+ \gamma \max_{a'}Q_\theta(s', a') - Q_\theta(s, a))^{2}ds'dads
\end{align*}
$$
- The **MSTD** measures the <mark style="background: #FFB86CA6;">average squared difference between the predicted Q-values and the observed rewards plus the discounted Q-values of the next state.</mark> It’s a way to quantify how off our predictions are on average.

- However, we want to minimize the Mean Squared Bellman Error (MSBE), because it ensures that the Bellman optimality condition is satisfied, which is <mark style="background: #FFB86CA6;">crucial for finding the optimal policy</mark>. 
$$\begin{align*}
\text{MSBE} &= \mathbb{E}_{\pi}[(r_{t} + \gamma \mathbb{E}_{p(\centerdot|s,a)}[\max_{a'} Q_\theta(s', a')] - Q_\theta(s, a))^2]\\
&=\int \mu^{\pi}(s)\int \pi(a|s) \left(\underbrace{ r_t + \gamma \int p(s'|s,a) \max_{a'} Q_\theta(s',a') ds' - Q_\theta(s,a) }_{\text{=\ 0 -> Bellman optimality condition}}\right)^2 da ds 
\end{align*}
$$
- But this has the following problems:
	1. **Model Dependency**: The inner expectation of the MSBE involves an expectation over the next state $s'$, which requires knowledge of the transition dynamics $p(s'|s,a)$. This can only be accurately evaluated if we have a model of the environment, which is often not available or too complex to be practical.
	2. **Multiple Samples Requirement**: To avoid the need for a model, one <mark style="background: #FFB86CA6;">could theoretically use multiple samples</mark> of $s'$ to estimate the expectation. However, this assumes that we can sample the next state $s'$ multiple times given the current state $s$ and action $a$, which is unrealistic in many real-world systems where each state transition is unique and cannot be repeated under identical conditions.
	3. **Bias with Single Sample**: Using only a single sample of $s'$ to estimate the expectation introduces bias. This is because the expectation is inside the square in the MSBE formula, and <mark style="background: #FFB86CA6;">using a single sample would actually yield the Mean Squared Temporal Difference (MSTD) error, which does not account for the variance of the value function across different states.</mark>
	4. **Double Sampling**: The text mentions that using two samples would work, referring to the double sampling problem. This means that if we could <mark style="background: #FFB86CA6;">independently sample two next states</mark> $s'$from the same $s$ and $a$, we could use one sample to estimate the maximum future value and the other to estimate the expectation, thus reducing bias. However, this is often not feasible in practice.

## Problem with Q-Learning with Neural Networks
- Can diverge easily, even if the optimal [[Value-Function]] could be represented by $Q_{\theta}(s,a)$
- A simple [[Markov Decision Process|MDP]] can be constructed where it diverges:
	- **State Space**: ( $S = {s_1, s_2}$ )
	- **Action Space**: ( $A = {a_1, a_2}$ )
	- **Transitions**: Taking action ( $a_1$ ) in state ( $s_1$ ) always leads to ( $s_2$), and taking action ( $a_2$ ) in state ( $s_2$ ) always leads back to ( $s_1$ ).
	- **Rewards**: A reward of +1 is given for taking action ( $a_1$ ) in ( $s_1$ ), and a reward of -1 for taking action ( $a_2$ ) in ( $s_2$ ).
	- Here’s the transition diagram for the MDP:
	```plaintext
	s_1 --a_1--> s_2 (+1 reward)
	s_2 --a_2--> s_1 (-1 reward)
	```
	In this MDP, the optimal policy is to always take action ( a_1 ) to maximize rewards. However, due to the strong correlation between sequential states and the constantly changing target values, a neural network might struggle to converge to the optimal Q-values using standard Q-Learning. This is because after each update, the estimated Q-values for the next state would change, leading to a different target value for the current state-action pair. Over time, this could cause the Q-values to oscillate or diverge instead of converging to the optimal values. To address these issues, techniques like experience replay and target networks are often used to stabilize training with neural networks in Q-Learning.

- Due to 2 main reasons:
1. **Sequential States Correlation**: In reinforcement learning, sequential states are often correlated because they result from actions taken in similar situations.When using neural networks for function approximation in Q-Learning, this correlation can lead to overfitting to recent sequences of states and actions, while forgetting learned knowledge of earlier states and actions -> [[Catastrophic Forgetting]]
	- Solution: [[Value-Function Approximation#Q-Learning with Replay Buffers|Q-Learning with Replay Buffers]]
2. **Non-Stationary Targets**: The target value in Q-Learning is the expected future rewards, which is constantly changing as the Q-values are updated. This non-stationarity can be problematic for neural networks, which are designed to learn from stationary distributions. As the Q-values update, the neural network must adapt to a moving target, which can lead to divergence or oscillations in the learned values.
	- Solution: [[Value-Function Approximation#Target Networks|Target Networks]]
### Q-Learning with Replay Buffers
- Keep a batch of trajectories collected for a different policy -> [[Off-Policy]]
- Update the parameters using samples from that batch
![[Pasted image 20240313130842.png#invert|600]]
- **Makes Data Distribution More Stationary**: Replay buffers randomize the data over which the neural network trains, breaking the correlation between consecutive samples
	- But we still have a moving target! Still unstable
- **Avoids Catastrophic Forgetting**: By storing past experiences and randomly sampling from them to train the network
- **Improves Data Efficiency**: Replay buffers allow the reuse of past experiences multiple times for training
### Q-Learning with Regression
![[Pasted image 20240313134929.png#invert|600]]
1. **Compute Targets**: For each tuple in the dataset, the target $y_i$ is computed using the formula: $y_i = r(s_i, a_i) + \gamma \max_{a'} Q_{\beta}(s'_i, a')$ where $\gamma$ is the discount factor and $Q_{\beta}$ is the Q-value function parameterized by $\beta$
2. **Solve Full Regression**: The parameters ( \beta ) of the Q-value function are updated by solving a regression problem that minimizes the sum of squared errors between the predicted Q-values and the computed targets: $\beta = \arg\min_{\beta} \sum ||Q_{\beta}(s_i, a_i) - y_i||^2$
This algorithm iteratively updates the Q-value function by fitting it to the computed targets, which represent the expected future rewards. The regression step helps to smooth out the Q-value function and can <mark style="background: #FFB86CA6;">lead to more stable learning compared to online Q-learning methods</mark>. The use of a replay buffer in this context helps to make the data distribution more stationary and improves data efficiency by reusing old experiences. 
- **Advantages**:
	- **Well-Defined Stable Regression**: The algorithm involves <mark style="background: #FFB86CA6;">a clear regression problem</mark> where the objective is to minimize the error between the predicted Q-values and the target values. This regression is stable because it uses a batch update method, meaning it updates the Q-values based on a batch of experiences rather than from a single new experience at a time. This can lead to more stable and reliable convergence to the optimal Q-values.
- **Disadvantages**:
	- **Computationally Costly**: Each iteration of the Fitted Q-Iteration algorithm <mark style="background: #FFB86CA6;">requires solving a full regression problem</mark>, which can be computationally intensive, especially as the size of the dataset grows. This is because the algorithm must process the entire batch of experiences to perform the update, which can be slow and impractical for large-scale problems or real-time applications.
### Target Networks
- Addresses the issue of moving targets
- The target network has the same architecture as the primary network but uses an older set of weights. These weights are updated less frequently, typically by slowly blending in the weights from the primary network at regular intervals.
![[Pasted image 20240313135626.png#invert|600]]
- Stability: By using an older set of weights to compute the targets, <mark style="background: #FFB86CA6;">the target function does not change with every single update</mark>. This adds stability to the learning process because the targets remain consistent over multiple updates.
- Reduced Feedback Loop: Since the target network changes slowly, the feedback loop caused by the similarity between $s$ and $s'$ is reduced. This prevents the amplification of changes in Q-values.
### Alternative Target Networks
- Aims to provide a more stable learning signal than the traditional approach of periodically copying the weights from the primary network
- Instead of directly copying the weights from the primary network, update the target network’s weights by <mark style="background: #FFB86CA6;">taking a moving average of the primary network’s weights</mark>. This can be done <mark style="background: #FFB86CA6;">using Polyak averaging</mark>, where the target network’s weights ( $\theta^-$ ) are updated as follows: $$\theta' \leftarrow \tau \theta' + (1 - \tau) \theta$$ Here, $\tau$ is a constant (e.g., 0.999), $\theta$ represents the primary network’s weights, and $\theta'$ represents the target network’s weights.
- The intuition behind this approach is to smooth out the updates to the target network, which can help mitigate the issue of moving targets and provide a more consistent learning signal. This <mark style="background: #FFB86CA6;">can lead to improved stability and performance</mark> in the training process. The “lag” mentioned refers to the delay between changes in the primary network’s weights and their reflection in the target network’s weights. By using a moving average, the lag is consistent across all time steps, which can be beneficial for learning.
## Deep Q-Learning Network (DQN)
- Combination of a NN as function approximator, Replay Buffer and Target Networks
![[Pasted image 20240313135852.png#invert|600]]
- Still, there is no proof of convergence as we change the targets and it might even diverge
- In practice, it often works well

- Can also be done using Alternative Target Networks
![[Pasted image 20240313140350.png#invert|600]]
- Extensions/Improvements to DQN include:
### Double Q-Learning
- This method addresses the issue of overestimation in Q-values by using two separate Q-networks. 
- One network is used to select the best action, and the other is used to evaluate the value of that action. 
- This helps to prevent the overestimation that can occur when a single network is used for both selection and evaluation.
$$Q_{\theta_{2}}(s_t,a_t)\leftarrow r(s_t,a_{t})+\gamma Q_{\theta_{1}}(s_{t+1}, arg\max_{a}Q_{\theta_{2}}(s_{t+1},a))$$$$
Q_{\theta_{1}}(s_t,a_t)\leftarrow r(s_t,a_{t})+\gamma Q_{\theta_{2}}(s_{t+1}, arg\max_{a}Q_{\theta_{1}}(s_{t+1},a))$$
- One Q-function, say $Q_{\theta_{1}}$, is used to select the best action based on the current state.
- The other Q-function, $Q_{\theta_{2}}$, is then used to evaluate the value of taking that action.
- The update rule for Double Q-Learning involves updating the Q-function that was not used for action selection with the reward and the discounted value of the next state as evaluated by the other Q-function.
- If the two Q’s are noisy in different ways, then there is no overestimation!

- In Double Q-Learning, the two Q-networks referred to are the **current (online) Q-network** and the **target Q-network**. Here’s how they are used:
	- **Current Q-Network**: This is the network that is being actively updated during the training process. It is used to evaluate and select the best action to take in a given state.
	- **Target Q-Network**: This network is a slightly older version of the current Q-network. Its weights are updated less frequently, typically by copying the weights from the current Q-network at regular intervals. The target network is used to evaluate the value of the action selected by the current network.
#### Overestimation Bias in Q-Values
- In Q-Learning, the max operator used to select the best action can lead to overestimation. 
- This happens because the noise in the Q-value estimates causes the maximum expected value to be higher than the true value. 
- Double Q-Learning mitigates this by having separate networks, which reduces the likelihood of overestimation if the networks are noisy in different ways.
### Multi-Step Returns
- This extension uses N-step returns instead of the standard 1-step return
- It combines rewards from multiple steps to update the Q-values, which can lead to less biased target values when Q-values are inaccurate.
- There’s a balance between bias and variance
	- 1-step returns have low variance but high bias
	- Monte-Carlo returns (T-step) have no bias but high variance.
	- N-step returns aim to find a middle ground.
$$y_{t}^{[n]}=\sum_{k=t}^{t+n}\gamma^{k-t}r(s_{k},a_{k})+\gamma^{n}\max_{a'}Q_{\theta'}(s_{t+n},a')$$
- <mark style="background: #FFB86CA6;">Using N-step returns can accelerate learning, especially in the early stages, because it provides less biased target values.</mark>
- Only works when learning [[On-Policy]]
	- Multi-step returns estimate the expected return for following the current policy for N steps. If you’re learning [[Off-Policy]], the actions taken may not be from the current policy, leading to incorrect estimates
	- Solutions:
		- Ignoring the Problem: Sometimes, even if the data includes off-policy actions, the algorithm can still learn effectively. This is because the bias introduced might not significantly affect the learning process.
		- Cutting the Trace: If an action is [[Off-Policy]], you can cut the trace there. This means you only consider rewards up to that point for updating the Q-value.
		- Dynamically Choosing N: By dynamically adjusting N, you can ensure that the data used for the N-step return is [[On-Policy]]. This works well when most of the data is [[On-Policy]] and the action space is small.
