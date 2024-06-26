> [!summary] 
> Instead of keeping a table of all values, we use a parameterized [[Value-Function]].
> 
- **Generalization**: Instead of trying to learn and store Q-values for every possible state-action pair, VFA aims to learn about a smaller number of states from experience and generalize this knowledge to new, similar states.
- **Function Approximators**: This is done using function approximators like neural networks, decision trees, or linear functions, which can estimate Q-values for states that haven’t been explicitly visited during training.
- **Machine Learning Principle**: This approach is a fundamental idea in machine learning, where we use a limited set of training data to learn a model that can generalize and make predictions on new, unseen data.

- **Representation**: VFA represents the [[Value-Function#State-Value Function/V-Function|V-Function]] or [[Value-Function#Action-Value Function/Q-Function|Q-Function]] with a parameterized function $Q_{\theta}(s, a)$, where $\theta$ represents the parameters of the function.
- **Learning**: The parameters $\theta$ are adjusted during the learning process to minimize the difference between the predicted Q-values and the target Q-values (which are based on the observed rewards and the estimated values of subsequent states).

## Advantages 
- Allows handling environments with large or continuous state spaces by providing a compact representation that can generalize across states and actions.
- Estimate the value of states and actions, rather than relying on a potentially unmanageable table of Q-values

## Fitting a Value Function
“Fitting” in the context of value functions refers to the process of adjusting the parameters of a model (value function) so that it best represents the relationship between states, actions, and rewards based on the data (experiences) collected from the environment.
1. **Fit Monte-Carlo Returns**:
    - This method involves using the [[Markov Decision Process#Return|Return]] from complete episodes to fit the value function by directly minimizing the regression loss:$$L=\sum_{t}(V_{\theta}(s_{t})-R_{t})^2$$
    - The idea is to run many episodes using the current policy, record the returns for each state-action pair, and then use these returns to estimate the value function directly.
    - Pros: 
        - Can use any supervised learning method (e.g. [[Backpropagation]]) 
        - Yields unbiased estimate of the Value Function (Approximates the real targets) 
        - Always converges (its just regression...) 
    - Cons:
        - High variance in returns $R_t$, therefore requires tons of training data
        - Can not use the last few transitions of an episode as training data
        - At least if we end the episode just because of the episode-length
2. **Q-Learning with Neural Networks**:
    - Neural networks can be used as function approximators to estimate the Q-values for state-action pairs.
    - In this approach, the neural network’s weights are adjusted to minimize the difference between the predicted Q-values and the target Q-values (calculated using the Bellman equation).
    - This method allows handling large or continuous state spaces and can generalize to unseen states and actions.
3. **Fitted Q-Iteration**:
    - Fitted Q-Iteration is a batch learning method where the Q-function is fitted using a dataset of transitions (state, action, reward, next state).
    - The algorithm iteratively updates the Q-values by fitting a model to predict the maximum future rewards for each state-action pair in the dataset.
    - It’s an off-policy method, meaning it can learn from data collected from a different policy than the one currently being optimized.

## Approximate Q-Learning
- Updating the Q-function with gradient descent
$$\begin{align*}
\theta_{\text{new}} &= \theta - \alpha \frac{d}{d\theta} L_t \\
&= \theta - \alpha \frac{d}{d\theta} (Q(\theta(s_t, a_t)) - y_t)^2\\
&= \theta - 2\alpha(Q(\theta(s_t, a_t)) - y_t) \frac{d}{d\theta} Q(\theta(s_t, a_t))\\
\end{align*}
$$
- Gradient Descent Update Rule: The parameters are updated in the direction that reduces the loss, which is the difference between the predicted Q-values and the target values (actual rewards plus the discounted value of the next state).
- Parameter Update with [[Temporal Difference Learning|TD Error]]: The parameters are updated by moving in the direction of the gradient of the Q-function scaled by the TD error. This ensures that the parameters are adjusted to reduce the prediction error.

$$\begin{align*}\theta_{\text{new}} &= \theta + \alpha(r(s'_t,a'_t) + \gamma \max_{a'}Q(\theta(s'_t+1,a')) - Q(\theta(s'_t,a'_t))) \frac{d}{d\theta}Q(\theta(s'_t,a'_t))\\
&= \theta + \alpha\delta_{t} \frac{d}{d\theta}Q_{\theta}(s'_t,a'_t)
\end{align*}$$
- $\frac{d}{d\theta}Q_{\theta}(s_t,a_t)$: The gradient of the Q-function with respect to its parameters. This tells us in which direction to change the parameters to increase the Q-value for the state-action pair $(s_t, a_t)$
- In approximate Q-learning, the gradient of the Q-function is multiplied with the TD error to update the parameter vector, which allows the model to learn from the experiences and improve the policy iteratively
![[Pasted image 20240313093558.png#invert|For simplicity, the terminal states are neglected]]
### Problem
- **No Clearly Defined Objective**: In many machine learning tasks, there is a clear objective function that you’re trying to optimize. For example, in supervised learning, you might minimize the difference between your predictions and the true labels. However, here the objective isn’t fixed because the targets (the optimal future values) are not known in advance and change as the agent learns.
- **Changing Targets**: The target values $y_{t}=r(s_{t},a_{t})+\gamma \max_{a'}Q^{\pi}(s_{t+1},a')$  in Q-learning are based on the current estimate of the future rewards. As the agent learns and updates its policy, these estimates change. This means that the ‘ground truth’ the agent is trying to learn from is a moving target, which complicates the learning process.
- **Similarity of Successive States**: In many environments, successive states $s$ and $s'$ are very similar to each other. This similarity means that when the agent updates its estimate for one state, it likely affects its estimates for similar states. This can lead to instability in the learning process because changing the estimate for one state changes the targets for many others.
- **No Gradient Computed**: Because the targets are constantly changing, you can’t compute a stable gradient to optimize as you would in other types of machine learning. The gradient is the direction in which you should adjust your parameters to improve your predictions, but if the ‘correct answers’ keep changing, it’s hard to know which way to go.

- **Intuition**: Imagine you’re trying to shoot an arrow at a target, but every time you shoot, the target moves based on where your last arrow landed. You’re trying to hit the bullseye, but since the target keeps moving, it’s hard to learn how to adjust your aim. In Q-learning, the moving target is the constantly updating estimate of future rewards, and the arrows are your actions. You’re trying to learn the best actions to take, but it’s challenging because the criteria for success are always changing.
### Solutions 1: Use the real gradient
-  Instead of using approximate gradients, this method uses the true gradient of the Q-function with respect to its parameters. This leads to a more accurate update rule, which is expected to result in better learning performance. $$\theta_{\text{new}} = \theta - \alpha \frac{d}{d\theta} L_t = \theta - \alpha \frac{d}{d\theta} (Q(\theta(s_t, a_t)) - y_t)^2
$$
- The **update rule** is about adjusting the parameters ( \theta ) of the Q-function to better predict the expected rewards for each state-action pair. The learning rate ( \alpha ) controls how much we adjust the parameters based on the error between our prediction and the actual outcome.
- This minimizes the Mean Squared TD Error (MSTD), which is a biased objective because it doesn’t lead to Bellman optimality. It’s calculated as the expectation of the squared difference between the sum of rewards and discounted future state-action values, and current state-action values.
$$\begin{align*}\\\\
\text{MSTD} &= \mathbb{E}_{\pi}[(r_{t} + \gamma \max_{a'}Q_\theta(s', a') - Q_\theta(s, a))^{2}\\
&= \int\mu^{\pi}(s)\int\pi(a|s)\int p(s'|s,a)(r_{t}+ \gamma \max_{a'}Q_\theta(s', a') - Q_\theta(s, a))^{2}ds'dads
\end{align*}
$$
- The **MSTD** measures the average squared difference between the predicted Q-values and the observed rewards plus the discounted Q-values of the next state. It’s a way to quantify how off our predictions are on average.

- However, we want to minimize the Mean Squared Bellman Error (MSBE), because it ensures that the Bellman optimality condition is satisfied, which is crucial for finding the optimal policy. 
$$\begin{align*}
\text{MSBE} &= \mathbb{E}_{\pi}[(r_{t} + \gamma \mathbb{E}_{p(\centerdot|s,a)}[\max_{a'} Q_\theta(s', a')] - Q_\theta(s, a))^2]\\
&=\int \mu^{\pi}(s)\int \pi(a|s) \left(\underbrace{ r_t + \gamma \int p(s'|s,a) \max_{a'} Q_\theta(s',a') ds' - Q_\theta(s,a) }_{\text{=\ 0 -> Bellman optimality condition}}\right)^2 da ds 
\end{align*}
$$
- But this has the following problems:
	1. **Model Dependency**: The inner expectation of the MSBE involves an expectation over the next state $s'$, which requires knowledge of the transition dynamics $p(s'|s,a)$. This can only be accurately evaluated if we have a model of the environment, which is often not available or too complex to be practical.
	2. **Multiple Samples Requirement**: To avoid the need for a model, one could theoretically use multiple samples of $s'$ to estimate the expectation. However, this assumes that we can sample the next state $s'$ multiple times given the current state $s$ and action $a$, which is unrealistic in many real-world systems where each state transition is unique and cannot be repeated under identical conditions.
	3. **Bias with Single Sample**: Using only a single sample of $s'$ to estimate the expectation introduces bias. This is because the expectation is inside the square in the MSBE formula, and using a single sample would actually yield the Mean Squared Temporal Difference (MSTD) error, which does not account for the variance of the value function across different states.
	4. **Double Sampling**: The text mentions that using two samples would work, referring to the double sampling problem. This means that if we could independently sample two next states $s'$from the same $s$ and $a$, we could use one sample to estimate the maximum future value and the other to estimate the expectation, thus reducing bias. However, this is often not feasible in practice.

## Problem with Q-Learning with Neural Networks
- Can diverge easily, even if the optimal [[Value-Function]] could be represented by $Q_{\theta}(s,a)$
- A simple [[Markov Decision Process|MDP]] can be constructed where it diverges:
	- **State Space**: ( S = {s_1, s_2} )
	- **Action Space**: ( A = {a_1, a_2} )
	- **Transitions**: Taking action ( a_1 ) in state ( s_1 ) always leads to ( s_2 ), and taking action ( a_2 ) in state ( s_2 ) always leads back to ( s_1 ).
	- **Rewards**: A reward of +1 is given for taking action ( a_1 ) in ( s_1 ), and a reward of -1 for taking action ( a_2 ) in ( s_2 ).
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
This algorithm iteratively updates the Q-value function by fitting it to the computed targets, which represent the expected future rewards. The regression step helps to smooth out the Q-value function and can lead to more stable learning compared to online Q-learning methods. The use of a replay buffer in this context helps to make the data distribution more stationary and improves data efficiency by reusing old experiences. 
- **Advantages**:
	- **Well-Defined Stable Regression**: The algorithm involves a clear regression problem where the objective is to minimize the error between the predicted Q-values and the target values. This regression is stable because it uses a batch update method, meaning it updates the Q-values based on a batch of experiences rather than from a single new experience at a time. This can lead to more stable and reliable convergence to the optimal Q-values.
- **Disadvantages**:
	- **Computationally Costly**: Each iteration of the Fitted Q-Iteration algorithm requires solving a full regression problem, which can be computationally intensive, especially as the size of the dataset grows. This is because the algorithm must process the entire batch of experiences to perform the update, which can be slow and impractical for large-scale problems or real-time applications.
### Target Networks
- Addresses the issue of moving targets
- The target network has the same architecture as the primary network but uses an older set of weights. These weights are updated less frequently, typically by slowly blending in the weights from the primary network at regular intervals.
![[Pasted image 20240313135626.png#invert|600]]
- Stability: By using an older set of weights to compute the targets, the target function does not change with every single update. This adds stability to the learning process because the targets remain consistent over multiple updates.
- Reduced Feedback Loop: Since the target network changes slowly, the feedback loop caused by the similarity between $s$ and $s'$ is reduced. This prevents the amplification of changes in Q-values.
### Alternative Target Networks
- Aims to provide a more stable learning signal than the traditional approach of periodically copying the weights from the primary network
- Instead of directly copying the weights from the primary network, update the target network’s weights by taking a moving average of the primary network’s weights. This can be done using Polyak averaging, where the target network’s weights ( \theta^- ) are updated as follows: $$\theta' \leftarrow \tau \theta' + (1 - \tau) \theta$$ Here, $\tau$ is a constant (e.g., 0.999), $\theta$ represents the primary network’s weights, and $\theta'$ represents the target network’s weights.
- The intuition behind this approach is to smooth out the updates to the target network, which can help mitigate the issue of moving targets and provide a more consistent learning signal. This can lead to improved stability and performance in the training process. The “lag” mentioned refers to the delay between changes in the primary network’s weights and their reflection in the target network’s weights. By using a moving average, the lag is consistent across all time steps, which can be beneficial for learning.
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
- Using N-step returns can accelerate learning, especially in the early stages, because it provides less biased target values.
- Only works when learning [[On-Policy]]
	- Multi-step returns estimate the expected return for following the current policy for N steps. If you’re learning [[Off-Policy]], the actions taken may not be from the current policy, leading to incorrect estimates
	- Solutions:
		- Ignoring the Problem: Sometimes, even if the data includes off-policy actions, the algorithm can still learn effectively. This is because the bias introduced might not significantly affect the learning process.
		- Cutting the Trace: If an action is [[Off-Policy]], you can cut the trace there. This means you only consider rewards up to that point for updating the Q-value.
		- Dynamically Choosing N: By dynamically adjusting N, you can ensure that the data used for the N-step return is [[On-Policy]]. This works well when most of the data is [[On-Policy]] and the action space is small.
### Distributional RL
- Focuses on estimating the **distribution** of expected returns, rather than just the expectation
- It uses a **distributional Bellman equation**, which is similar to the regular Bellman operator but works with distributions.
- Advantages:
	- Can be learned by [[Loss Function#Categorical Cross-Entropy Loss (CCE)|cross-entropy-loss]]
	- Less sensitive to outliers that [[Loss Function#Mean Squared Error (MSE)|MSE]]
	- 