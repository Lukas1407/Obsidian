> [!summary] Definition
> Policy optimization is a family of algorithms in reinforcement learning (RL) that directly optimize the [[Policy]] without necessarily relying on a v[[Value-Function]].
> 

- Policy optimization algorithms directly adjust the policy parameters to maximize the expected return
	- This can be simpler than learning the [[Value-Function#State-Value Function/V-Function|V-Function]] or [[Value-Function#Action-Value Function/Q-Function|Q-Function]]

## Policy Gradient Methods
- These methods use a policy that is parameterized by a set of weights, which can be adjusted to change the behavior of the policy.
- $$\pi_{\theta}(a|s)$$
	- For example: Gaussian Neural Network Policy:$$\pi_{theta}(a|s)=\mathcal{N}(a|\mu_{\theta}(s), \Sigma_{\theta}(s))$$
	- Works well for continuous actions compared to discrete actions like [[Exploration and Entropy#Using Neural Networks|softmax-policy]]
- The policy is typically stochastic, meaning it assigns probabilities to actions rather than deterministically choosing a single action. This inherent randomness helps in exploring the environment
- The parameters of the policy are updated using gradient ascent on the expected return. This means that the algorithm calculates the gradient of the expected return with respect to the policy parameters and then moves the parameters in the direction that increases the expected return
	- Gradient ascent because we want to maximize the return and not minimize it like in [[Gradient Descent]]
		- $$\theta \leftarrow\theta+\alpha \frac{\partial J\theta}{\partial\theta}$$, with the objective function $J$  that is being optimized is typically the expected return:$$J(\theta)=\mathbb{E}_{\tau\sim p_{\theta}(\tau)}\left[\sum_{t}\gamma^{t}r(s_{t},a_{t}) \right]$$, $\tau$ being a trajectory, and $p_{\theta}(\tau)$ is the trajectory distribution induced by policy parameter $\theta$:$$p_{\theta}(\tau)=p(s_{0})\prod_{t=0}^{T-1}\pi_{\theta}(a_{t}|s_{t})p(s_{t+1}|s_{t},a_{t})$$
![[Pasted image 20240314083649.png#invert|400]]
### Goal
- Find the best parameters for that policy $$\theta^{*}=arg\max_{\theta}\mathbb{E}_{\tau\sim p_{\theta}}(\tau)\left[\sum_{t}^{T}\gamma^{t}r(s_{t},a_{t}) \right]$$
### Computing the Gradient
- Objective: $$J(\theta)=\mathbb{E}_{\tau\sim p_{\theta}(\tau)}\left[\sum_{t}\gamma^{t}r(s_{t},a_{t}) \right]=\int p_{\theta}(\tau)R(\tau)d\tau$$
- Gradient:$$\nabla_{\theta}J(\theta)=\int\nabla_{\theta}p_{\theta}(\tau)R(\tau)d\tau = \int \underbrace{p_{\theta}(\tau)\nabla_{\theta}\log p_{\theta}(\tau) R(\tau)d\tau}_{\text{Log-ratio trick}}$$
	- Uses the [[Log-Ratio Trick]]
	- Can be evaluated via sampling ([[Monte-Carlo Estimation]]):$$\nabla_{\theta}J(\theta)\approx\frac{1}{N}\sum_{i}\left(\underbrace{\sum_{t}\nabla_{\theta}\log \pi_{\theta}(a_{i,t}|s_{i,t})}_{\text{Gradient of Log-Likelihood}} \right)\left(\underbrace{\sum_{t}\gamma^{t}r(s_{i,t}, a_{i,t})}_{\text{Return}}\right)$$
		- Called likelihood-Ratio Policy Gradients
		- Valid even when the reward is discontinuous and/or unknown or when sample space of paths ($\tau$) is a discrete set
	- The gradient tries to: 
		- Increase probability of paths with high returns
		- Decrease probability of paths with low returns
	- Does not try to change the paths (<-> path derivative): 
		- The method doesn’t directly modify the paths themselves; it only changes the policy that generates these paths. In other words, it doesn’t compute the derivative of the paths (which would require knowing how the paths would change with respect to the policy parameters), but rather the derivative of the probability of taking a particular action given a state.
		- We would need the transition model for the path derivatives: To compute the path derivatives, you would need a model of the environment’s dynamics, known as the transition model, which describes the probability of moving to the next state given the current state and action. This is typically not required in policy gradient methods, which are often model-free.
- How to compute $\nabla_{\theta}\log p_{\theta}(\tau)$:$$\begin{align}p_{\theta}(\tau)&=p(s_{0})\prod_{t=0}^{T-1}\pi_{\theta}(a_{t}|s_{t})p(s_{t+1}|s_{t},a_{t}) \\ \log p_{\theta}(\tau)&=\log p(s_{0})+\sum_{t=0}^{T-1}\log \pi_{\theta}(a_{t}|s_{t})+\sum_{t=0}^{T-1}\log p(s_{t+1}|s_{t},a_{t}) \\ \nabla_{\theta}\log p_{\theta}(\tau)&= \nabla_{\theta}\log p(s_{0})+\sum_{t=0}^{T-1}\nabla_{\theta}\log \pi_{\theta}(a_{t}|s_{t})+\sum_{t=0}^{T-1}\nabla_{\theta}\log p(s_{t+1}|s_{t},a_{t}) \\ &=\sum_{t=0}^{T-1}\nabla_{\theta}\log \pi_{\theta}(a_{t}|s_{t}) \end{align}$$
	- -> No model is required!
### REINFORCE Algorithm
![[Pasted image 20240315084047.png#invert|600]]
- Converges to local optimum of J
- But: 
	- Learning rates very hard to tune 
	- Needs tons of samples

## Extensions
As formulated thus far: 
1. Unbiased but very noisy (high variance) 
	1. Needs small learning rate 
	2. Slow convergence 
 2. Can only be used on-policy
	 1. No data re-use 
3. Very hard to tune learning rates
### Temporal Structure
- Unrolls the temporal structure of the gradient: $$\begin{align}\nabla_{\theta}J(\theta)&\approx\frac{1}{N}\sum_{i}\left(\sum_{t}\nabla_{\theta}\log \pi_{\theta}(a_{i,t}|s_{i,t})\right)\left(\sum_{t}\gamma^{t}r(s_{i,t}a,_{i,t})\right)\\ &= \frac{1}{N}\sum_{i}\left(\sum_{t}\nabla_{\theta}\log \pi_{\theta}(a_{i,t}|s_{i,t})\right)\left(\sum_{k=0}^{t-1}\gamma^{k}r(s_{i,k},a_{i,k}) + \sum_{k=t}^{T}\gamma^{k}r(s_{i,k},a_{i,k})\right)\end{align}$$
- Past rewards $k < t$ do not depend on $a_{i,t}$: 
	- $\sum_{k=0}^{t-1}\gamma^{t}r(s_{i,k},a_{i,k})$
	- Rewards received before time $t$ cannot be influenced by the action taken at time $t$, because those rewards have already been determined by previous actions. In other words, the outcome of past actions cannot be changed by current or future actions.
- Future rewards $k \ge t$) do depend on $a_{i,t}$: 
	- $\sum_{k=t}^{T}\gamma^{t}r(s_{i,k},a_{i,k})$
	- The action $a_{i,t}$ can influence the state transitions and the rewards received from time $t$ onwards. Therefore, future rewards are dependent on the current action, as it can alter the trajectory of states and subsequent rewards.
- -> Terms that do not depend on $a_{i,t}$ can be removed!
	- This is important because it ensures that the policy is updated based on the potential for future rewards rather than being biased by the rewards already obtained.
	- Correlation between action selection strategy and past reward is zero $$\mathbb{E}[r_h \nabla_\theta \log \pi(a_t|s_t)] = 0 \text{ for } h < t$$
	- This ensures that the updates to the policy parameters are not influenced by rewards from the past, allowing the policy to focus on maximizing future rewards
- <mark style="background: #FFB86CA6;">The resulting equation is called Policy Gradient Theorem</mark>:
![[Pasted image 20240315085901.png#invert|800]]
### Baselines
- Subtracting a baseline $b$ from the returns $R_{i}$ reduces the variance in the returns
- $$\nabla_{\theta}J(\theta) = \frac{1}{N}\sum_{i=1}^{N}\nabla_{\theta}\log p_{\theta}(\tau_{i})(R_{i}-\textcolor{orange}{b})$$
	- By introducing a baseline, you’re effectively normalizing the returns
	- Since the baseline is subtracted from both positive and negative returns, it reduces the range of the values, thereby reducing the variance. 
	- The key point is that the baseline itself is independent of the actions, so subtracting it does not introduce any bias in the gradient estimate—it only affects the variance:$$\mathbb{E}_{p_{\theta}(\tau)}\left[\nabla_{\theta}\log p_{\theta}(\tau)\textcolor{orange}{b}\right]=\textcolor{orange}{b}\nabla_{\theta}\int p_{\theta}(\tau) d\tau=0$$
![[Pasted image 20240315090645.png#invert|200]]
#### Good choices of Baselines
1. Constant baselines: average Return $$b=\mathbb{E}_{p_{\theta}(\tau)}[R(\tau)]\approx \frac{1}{N}\sum_{i=1}^{T}R(\tau_{i})$$
2. Time-dependent baseline: expected reward to come from time $t$$$b_{t}=\mathbb{E}_{p_{\theta}(\tau)}\left[\sum_{k=t}^{T}\gamma^{k-t}r(s_k,a_k)\right]\approx \frac{1}{N}\sum_{i=1}^T\sum_{k=t}^{T}\gamma^{k-t}r(s_{i,k},a_{i,k})$$
	- This baseline adjusts for the fact that later actions do not affect earlier rewards, and it helps in reducing the variance more effectively than a constant baseline.
3. Sate-dependent baseline: expected reward to come $s$$$b(s)=\mathbb{E}_{p_{\theta}(\tau)}\left[\sum_{t=0}^{T}\gamma^{t}r(s_{t},a_{t})|s_{0}=s \right]=V^{\pi_{old}}(s)$$
	- This baseline is particularly effective because it accounts for the varying potential of different states to generate rewards.
	- Intuition: Increase log-prob of action proportionally to how much its returns are better than the expected return under the current policy
	- $V^{\pi_{old}}(s)$ indicates that the value function is evaluated under the policy $\pi_{old}$, not the current policy $\pi_{\theta}$. This distinction is important because the value function should reflect the expected returns under the policy that was used to generate the data (the sampling policy), not the policy that is currently being updated. This ensures that the baseline is properly calibrated and can effectively reduce the variance of the gradient estimates without introducing bias.
### Estimating the Q-Values
$$\begin{align}\nabla_{\theta}J(\theta)&\approx\frac{1}{N}\sum_{i}\left(\sum_{t}\nabla_{\theta}\log \pi_{\theta}(a_{i,t}|s_{i,t})\right)\left( \underbrace{\sum_{k=t}^{T}\gamma^{k-t}r(s_{i,k},a_{i,k})}_{\hat Q^{\pi_{old}}(s_{i,t},a_{i,t})}- V^{\pi_{old}}(s_{i,t}) \right )\end{align}$$
- $\hat Q^{\pi_{old}}(s_{i,t},a_{i,t})$: Estimation of expected reward to come if in state $s_{i,t}$ and executing action $a_{i,t}$ and following the sample policy $\pi_{old}$
- $\hat Q^{\pi_{old}}(s_{i,t},a_{i,t}) = \sum_{k=t}^{T}\gamma^{k-t}r(s_{i,k},a_{i,k})$ is a noisy version of the [[Value-Function#Action-Value Function/Q-Function|Q-Function]]:$$\hat Q^{\pi_{old}}(s_{i,t},a_{i,t})\approx\mathbb{E}_{\pi_{old}}\left[\sum_{k=t}^{T}\gamma^{k-t}r(s_{k},a_{k})|s_{k=t}=s_{i,t}, a_{k=t}=a_{i,t}\right]=Q^{\pi_{old}}(s_{i,t},a_{i,t})$$
	- Only uses a single sample / trajectory to estimate expectation
- How can we improve the estimate of $\hat Q^{\pi_{old}}(s_{i,t},a_{i,t})$?:
#### N-Step returns
N-step returns are a way to estimate the Q-function by looking ahead $n$ steps into the future rather than just one step (as in the standard Q-learning algorithm). This method balances the bias-variance trade-off: one-step returns have low variance but may be biased if the value function is not accurate, while returns that consider the entire episode ([[Monte-Carlo Estimation]]) are unbiased but can have high variance. $n$-step returns strike a middle ground by considering $n$ steps, which can lead to more stable and accurate value estimates.
$$\hat{Q}_{n}^{\pi_{old}}(s_t, a_t) = \sum_{k=t}^{t+n}\gamma^{k-t}r(s_{i,t},a_{i,t})+\gamma^nV^{\pi_{old}}(s_{i,t+n})
$$
- We “cut” the trajectories after $n$ steps and use the value function for predicting the future reward
- Small $n$: Reduced variance + Potential high bias (value function might be wrong) 
- Large $n$: Higher variance (each step adds variance) + Reduced bias
#### Generalized Advantage Estimation (GAE)
- Leverages the idea of n-step returns but instead of choosing a fixed n, it takes a weighted average of all n-step returns$$\hat{Q}_{GAE}^{\pi_{old}}(s_{i,t}, a_{i,t}) = \textcolor{orange}{(1-\lambda)}\sum_{n=1}^{\infty}\lambda^{n}\hat{Q}_{n}^{\pi_{old}}(s_{i,t}, a_{i,t})$$, with the normalization factor such that $(1-\lambda)\sum_{n=1}^{\infty}\lambda^{n}=1$
- The weights decrease exponentially for returns that are further in the future, which is controlled by a parameter $\lambda$
	-  This is known as the exponential weighting factor
- GAE uses the <mark style="background: #FFB86CA6;">advantage function</mark> $$A^{\pi}(s, a) = Q^{\pi}(s, a) - V^{\pi}(s)$$, which measures how much better taking a particular action $a$ is compared to the average action in state $s$ as per the current policy. ^bc0da1
	- -> is essentially the expected [[Temporal Difference Learning|TD error]] when action $a$ is taken in state $s$
	- In the context of GAE, the advantage at time %t is estimated as the sum of exponentially-discounted [[Temporal Difference Learning|TD errors]] 
- If we average over all possible actions $a$ weighted by their probability under the current policy, the advantage function averages out to zero:$$\mathbb{E}_{\pi(a|s)}[A^{\pi}(s,a)]=\mathbb{E}_{\pi(a|s)}[Q^{\pi}(s,a)]-V^{\pi}(s)=V^\pi(s)-V^{\pi}(s)=0$$
	- -> the advantage function has always zero mean
### Data Reuse
- The policy gradient is only valid if samples have been generated by $\pi$
	- -> we have to generate new trajectories after every gradient step
- Solution Importance Sampling
#### Importance Sampling
Importance Sampling allows us to estimate properties of a particular distribution, while only having samples generated from a different distribution
- The equation for importance sampling is typically written as:$$\hat{\mu}_{q}=\frac{1}{N}\sum_{x_{i}\sim q(x)}\frac{p(x_{i})}{q(x_{i})}f(x_{i})\approx\mathbb{E}_{p(x)}[f(x)]$$
- Here, $\hat{\mu}_{q}$ is the estimated expected value of a function $f(x)$ under the probability distribution $p(x)$, using samples ${x_i}$ from a different distribution $q(x)$.
- The term $\frac{p(x_i)}{q(x_i)}$ is known as the importance weight, and it corrects for the fact that the samples are drawn from $q(x)$ instead of $p(x)$.
- Disadvantages:
	- High variance if p and q are very different (the proposal distribution $q(x)$ is not a good approximation of the target distribution $p(x)$)
	- Basically only one sample gets all the weight
![[Pasted image 20240403091135.png#invert|300]]
- The expectation of a constant is incorrectly estimated due to the weights
#### Self-normalized Importance sampling
- This failure case can be fixed by normalizing the importance weights
- This is done by dividing each weight by the sum of all weights, which ensures that the weights sum up to one$$\mathbb{E}_{p(x)}[f(x)] \approx\frac{1}{Z}\sum_{x_{i}\sim q(x)}\frac{p(x_{i})}{q(x_{i})}f(x_{i})$$, with $$Z=\sum_{i} \frac{p(x_{i})}{q(x_{i})}$$
- Advantages:
	- With an infinite number of samples, the self-normalized estimator will converge to the true expectation under the target distribution $p(x)$
	- Normalizing the weights typically results in less variance compared to standard importance sampling because it mitigates the effect of any single sample having an outsized impact on the estimate
- Disadvantages:
	- Introduces bias, particularly with a finite number of samples. This is because the normalization process itself alters the distribution of the weights, which can lead to an estimate that is systematically off from the true value

## Policy Gradient Algorithm
![[Pasted image 20240403091906.png#invert|]]
## Gradient Step Size Regulation
Typically, we also learn the exploration noise of the policy
- The policies often start with high variance to encourage exploration. As learning progresses, the variance is reduced to refine the policy.
- Learning correlations between actions can lead to more directed and efficient exploration.
<mark style="background: #FF5582A6;">Main Problems:</mark>
- <mark style="background: #FFB86CA6;">Varying Gradient Magnitude</mark>: The magnitude of the policy gradient can change significantly during learning, making it difficult to choose an appropriate learning rate.
	-  A larger gradient magnitude means larger steps, and vice versa.
- <mark style="background: #FFB86CA6;">Quick Reduction in Exploration</mark>: If the exploration noise (covariance) decreases too quickly, it can lead to premature convergence to sub-optimal solutions.
Impact on Step Size:
- <mark style="background: #FFB86CA6;">Inverse Scaling</mark>: The gradient magnitude scales inversely with the variance of the policy. As the variance decreases, the gradient magnitude increases.
	- Variance of the Policy: This represents how much randomness there is in the policy’s action selection. A high variance means the policy is more exploratory, taking a wide range of actions. A low variance indicates a more deterministic policy, favoring certain actions.
	- When the variance is high, the policy explores more, and the gradient estimates are less reliable due to the high variability in the outcomes. Therefore, the algorithm takes smaller steps to avoid overfitting to noisy data.
	- As learning progresses and the variance decreases, the policy becomes more confident about which actions are better. This increased confidence is reflected in larger gradient magnitudes, allowing for more significant updates to the policy parameters.
	- Mathematically: $$\begin{split} \pi_{\theta}(a|s)=\mathcal{N}(a|f_{\theta}(s),\sigma^{2}) \\ \nabla_{\theta}\log\pi_{\theta}(a|s)=\frac{a-f_{\theta}(s)}{\sigma^{2}}\nabla_{\theta}f(s) \end{split}$$
		- As the variance $\sigma^{2}$ decreases, the value of the gradient $\nabla_{\theta}\log\pi_{\theta}(a|s)$ increases. This is because the term $\frac{1}{\sigma^{2}}$ becomes larger as $\sigma^{2}$ gets smaller.
- Large Step Size: A too large step size can cause the policy to jump to unexplored regions of the action space, where there is little data, potentially resulting in poor performance.
- Small Step Size: Conversely, a too small step size can lead to inefficient use of experience, requiring many samples to learn effectively.
Solution: Trust Regions
### Trust Regions
- Idea: keep the new policy within a certain “distance” (the trust region) from the old policy. This is done to ensure that the updates to the policy are not too drastic, which could lead to unstable learning or poor performance.
- $$\begin{split} \theta_{new}=\arg\max_{\theta}\mathbb{E}_{(s,a)\sim\pi_{old}(a|s)}\left[\frac{\pi_{\theta}(a|s)}{\pi_{old}(a|s)}\hat A^{\pi}(s,a)\right]\\ s.t. \ \mathcal{D}(\pi_{\theta}(a|s),\pi_{old}(a|s))\le\epsilon \end{split}$$ ^d61aef
- D represents a dissimilarity measure between the probability distributions of the old and new policies. The goal is to maximize the objective function while ensuring that the new policy does not deviate from the old one by more than a specified threshold $\epsilon$
	- Often uses the [[Kullback-Leiber Divergence]]
- Advantages: 
	- This approach stabilizes the learning process by preventing extreme changes in the policy.
	- It helps to maintain a sufficient level of exploration by avoiding a too rapid decrease in variance.
	- Monotonic improvement guarantees can be established, meaning that the performance of the policy is guaranteed to improve or remain the same with each update.
- How to solve a contained optimization problem? -> [[Lagrangian Multipliers]]:
![[Pasted image 20240404093917.png#invert|700]]
![[Pasted image 20240404094039.png#invert|700]]
- <mark style="background: #FF5582A6;">But</mark>: only valid for discrete actions and no states
- How can we do this for parametric policies such as Neural Networks?
	1. [[Natural Gradients]] and Trust-Region Policy Optimization ([[Natural gradients and Trust-Region Policy Optimization (TRPO)|TRPO]])
	2. [[Proximal Policy Optimization (PPO)]]
	3. [[Differential Trust Region Layers]]