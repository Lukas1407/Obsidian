> [!summary] Definition
> Policy optimization is a family of algorithms in reinforcement learning (RL) that directly optimize the [[Policy]] without necessarily relying on a [[Value-Function]].
> 

- Policy optimization algorithms directly adjust the policy parameters to maximize the expected return
	- This can be simpler than learning the [[Value-Function#State-Value Function/V-Function|V-Function]] or [[Value-Function#Action-Value Function/Q-Function|Q-Function]]

## Policy Gradient Methods
- These methods use a policy that is parameterized by a set of weights, which can be adjusted to change the behavior of the policy.
- $$\pi_{\theta}(a|s)$$
	- For example: Gaussian Neural Network Policy:$$\pi_{\theta}(a|s)=\mathcal{N}(a|\mu_{\theta}(s), \Sigma_{\theta}(s))$$
	- Works well for continuous actions compared to discrete actions like [[Exploration and Entropy#Using Neural Networks|softmax-policy]]
- The policy is typically stochastic, meaning it assigns probabilities to actions rather than deterministically choosing a single action. This inherent randomness helps in exploring the environment
- The parameters of the policy are <mark style="background: #FFB86CA6;">updated using gradient ascent on the expected return</mark>. This means that the algorithm calculates the gradient of the expected return with respect to the policy parameters and then moves the parameters in the direction that increases the expected return
	- Gradient ascent because we want to maximize the return and not minimize it like in [[Gradient Descent]]
		- $$\theta \leftarrow\theta+\alpha \frac{\partial J\theta}{\partial\theta}$$, with the objective function $J$  that is being optimized is typically the expected return:$$J(\theta)=\mathbb{E}_{\tau\sim p_{\theta}(\tau)}\left[\sum_{t}\gamma^{t}r(s_{t},a_{t}) \right]$$, $\tau$ being a trajectory, and $p_{\theta}(\tau)$ is the trajectory distribution induced by policy parameter $\theta$:$$p_{\theta}(\tau)=p(s_{0})\prod_{t=0}^{T-1}\pi_{\theta}(a_{t}|s_{t})p(s_{t+1}|s_{t},a_{t})$$
![[Pasted image 20240314083649.png#invert|400]]
### Goal
- Find the best parameters for that policy $$\theta^{*}=arg\max_{\theta}\mathbb{E}_{\tau\sim p_{\theta}}(\tau)\left[\sum_{t}^{T}\gamma^{t}r(s_{t},a_{t}) \right]$$
### Computing the Gradient
- Objective: $$J(\theta)=\mathbb{E}_{\tau\sim p_{\theta}(\tau)}\left[\sum_{t}\gamma^{t}r(s_{t},a_{t}) \right]=\int p_{\theta}(\tau)R(\tau)d\tau\approx \frac{1}{N}\sum_{i}\sum_{t}\gamma^{t}r(s_{i,t},a_{i,t})$$
- Gradient:$$\nabla_{\theta}J(\theta)=\int\nabla_{\theta}p_{\theta}(\tau)R(\tau)d\tau = \int \underbrace{p_{\theta}(\tau)\nabla_{\theta}\log p_{\theta}(\tau) R(\tau)d\tau}_{\text{Log-ratio trick}}$$
	- Uses the [[Log-Ratio Trick]]
	- Can be evaluated via sampling ([[Monte-Carlo Estimation]]):$$\nabla_{\theta}J(\theta)\approx\frac{1}{N}\sum_{i}\left(\underbrace{\sum_{t}\nabla_{\theta}\log \pi_{\theta}(a_{i,t}|s_{i,t})}_{\text{Gradient of Log-Likelihood}} \right)\left(\underbrace{\sum_{t}\gamma^{t}r(s_{i,t}, a_{i,t})}_{\text{Return}}\right)$$
		- Called likelihood-Ratio Policy Gradients
		- Valid even when the reward is discontinuous and/or unknown or when sample space of paths ($\tau$) is a discrete set
	- The gradient tries to: 
		- <mark style="background: #FFB86CA6;">Increase probability of paths with high returns</mark>
		- <mark style="background: #FFB86CA6;">Decrease probability of paths with low returns</mark>
	- Does not try to change the paths (<-> path derivative): 
		- The method <mark style="background: #FFB86CA6;">doesn’t directly modify the paths themselves</mark>; it only changes the policy that generates these paths. In other words, it doesn’t compute the derivative of the paths (which would require knowing how the paths would change with respect to the policy parameters), but rather the derivative of the probability of taking a particular action given a state.
		- We would need the transition model for the path derivatives: <mark style="background: #FFB86CA6;">To compute the path derivatives, you would need a model of the environment’s dynamics</mark>, known as the transition model, which describes the probability of moving to the next state given the current state and action. This is typically not required in policy gradient methods, which are often model-free.
- How to compute $\nabla_{\theta}\log p_{\theta}(\tau)$:$$\begin{align}p_{\theta}(\tau)&=p(s_{0})\prod_{t=0}^{T-1}\pi_{\theta}(a_{t}|s_{t})p(s_{t+1}|s_{t},a_{t}) \\ \log p_{\theta}(\tau)&=\log p(s_{0})+\sum_{t=0}^{T-1}\log \pi_{\theta}(a_{t}|s_{t})+\sum_{t=0}^{T-1}\log p(s_{t+1}|s_{t},a_{t}) \\ \nabla_{\theta}\log p_{\theta}(\tau)&= \nabla_{\theta}\log p(s_{0})+\sum_{t=0}^{T-1}\nabla_{\theta}\log \pi_{\theta}(a_{t}|s_{t})+\sum_{t=0}^{T-1}\nabla_{\theta}\log p(s_{t+1}|s_{t},a_{t}) \\ &=\sum_{t=0}^{T-1}\nabla_{\theta}\log \pi_{\theta}(a_{t}|s_{t}) \end{align}$$
	- -> No model is required!
## REINFORCE Algorithm
![[Pasted image 20240315084047.png#invert|600]]
- Converges to local optimum of $J$
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
	- Rewards received before time $t$ cannot be influenced by the action taken at time $t$, because those rewards have already been determined by previous actions. In other words, the <mark style="background: #FFB86CA6;">outcome of past actions cannot be changed by current or future actions</mark>.
- Future rewards $k \ge t$ do depend on $a_{i,t}$: 
	- $\sum_{k=t}^{T}\gamma^{t}r(s_{i,k},a_{i,k})$
	- The action $a_{i,t}$ can influence the state transitions and the rewards received from time $t$ onwards. Therefore, <mark style="background: #FFB86CA6;">future rewards are dependent on the current action</mark>, as it can alter the trajectory of states and subsequent rewards.
- -> Terms that do not depend on $a_{i,t}$ can be removed!
	- This is important because it ensures that the policy is updated based on the potential for future rewards rather than being biased by the rewards already obtained.
	- Correlation between action selection strategy and past reward is zero $$\mathbb{E}[r_h \nabla_\theta \log \pi(a_t|s_t)] = 0 \text{ for } h < t$$
	- This <mark style="background: #FFB86CA6;">ensures that the updates to the policy parameters are not influenced by rewards from the past, allowing the policy to focus on maximizing future rewards</mark>
- <mark style="background: #FFB86CA6;">The resulting equation is called Policy Gradient Theorem</mark>:
![[Pasted image 20240315085901.png#invert|800]]
### Baselines
- Subtracting a baseline $b$ from the returns $R_{i}$ reduces the variance in the returns
- $$\nabla_{\theta}J(\theta) = \frac{1}{N}\sum_{i=1}^{N}\nabla_{\theta}\log p_{\theta}(\tau_{i})(R_{i}-\textcolor{orange}{b})$$
	- By introducing a baseline, you’re effectively <mark style="background: #FFB86CA6;">normalizing the returns</mark>
	- Since the baseline is subtracted from both positive and negative returns, it <mark style="background: #FFB86CA6;">reduces the range of the values, thereby reducing the variance. </mark>
	- The key point is that the baseline itself is <mark style="background: #FFB86CA6;">independent of the actions</mark>, so subtracting it does not introduce any bias in the gradient estimate—it only affects the variance:$$\mathbb{E}_{p_{\theta}(\tau)}\left[\nabla_{\theta}\log p_{\theta}(\tau)\textcolor{orange}{b}\right]=\textcolor{orange}{b}\nabla_{\theta}\int p_{\theta}(\tau) d\tau=0$$
![[Pasted image 20240315090645.png#invert|200]]
#### Good choices of Baselines
1. <mark style="background: #FFB86CA6;">Constant baselines</mark>: average Return $$b=\mathbb{E}_{p_{\theta}(\tau)}[R(\tau)]\approx \frac{1}{N}\sum_{i=1}^{T}R(\tau_{i})$$
2. <mark style="background: #FFB86CA6;">Time-dependent baseline</mark>: expected reward to come from time $t$$$b_{t}=\mathbb{E}_{p_{\theta}(\tau)}\left[\sum_{k=t}^{T}\gamma^{k-t}r(s_k,a_k)\right]\approx \frac{1}{N}\sum_{i=1}^T\sum_{k=t}^{T}\gamma^{k-t}r(s_{i,k},a_{i,k})$$
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
N-step returns are a way to estimate the Q-function by looking ahead $n$ steps into the future rather than just one step (as in the standard Q-learning algorithm). 
- Balances the bias-variance trade-off: 
	- <mark style="background: #FFB86CA6;">one-step returns have low variance but may be biased</mark> if the value function is not accurate
	- returns that consider the entire episode ([[Monte-Carlo Estimation]]) are <mark style="background: #FFB86CA6;">unbiased but can have high variance</mark>. 
	- $n$-step returns strike a middle ground by considering $n$ steps, which can lead to <mark style="background: #FFB86CA6;">more stable and accurate value estimates</mark>.
$$\hat{Q}_{n}^{\pi_{old}}(s_t, a_t) = \sum_{k=t}^{t+n}\gamma^{k-t}r(s_{i,t},a_{i,t})+\gamma^nV^{\pi_{old}}(s_{i,t+n})
$$
- We “cut” the trajectories after $n$ steps and use the value function for predicting the future reward
- Small $n$: Reduced variance + Potential high bias (value function might be wrong) 
- Large $n$: Higher variance (each step adds variance) + Reduced bias
#### Generalized Advantage Estimation (GAE)
- Leverages the idea of n-step returns but <mark style="background: #FFB86CA6;">instead of choosing a fixed n, it takes a weighted average of all n-step returns</mark>$$\hat{Q}_{GAE}^{\pi_{old}}(s_{i,t}, a_{i,t}) = \textcolor{orange}{(1-\lambda)}\sum_{n=1}^{\infty}\lambda^{n}\hat{Q}_{n}^{\pi_{old}}(s_{i,t}, a_{i,t})$$, with the normalization factor such that $(1-\lambda)\sum_{n=1}^{\infty}\lambda^{n}=1$
- <mark style="background: #FFB86CA6;">The weights decrease exponentially for returns that are further in the future</mark>, which is controlled by a parameter $\lambda$
	-  This is known as the exponential weighting factor
- GAE uses the <mark style="background: #FFB86CA6;">advantage function</mark> $$A^{\pi}(s, a) = Q^{\pi}(s, a) - V^{\pi}(s)$$, which measures how much better taking a particular action $a$ is compared to the average action in state $s$ as per the current policy. ^bc0da1
	- -> is essentially the expected [[Temporal Difference Learning|TD error]] when action $a$ is taken in state $s$
	- In the context of GAE, the advantage at time t is estimated as the sum of exponentially-discounted [[Temporal Difference Learning|TD errors]] 
- If we average over all possible actions $a$ weighted by their probability under the current policy, the advantage function averages out to zero:$$\mathbb{E}_{\pi(a|s)}[A^{\pi}(s,a)]=\mathbb{E}_{\pi(a|s)}[Q^{\pi}(s,a)]-V^{\pi}(s)=V^\pi(s)-V^{\pi}(s)=0$$
	- -> the advantage function has always <mark style="background: #FFB86CA6;">zero mean</mark>
### Data Reuse
- The policy gradient is only valid if samples have been generated by $\pi$
	- -> we have to generate new trajectories after every gradient step
- Solution Importance Sampling
#### Importance Sampling
Importance Sampling allows us to <mark style="background: #FFB86CA6;">estimate properties of a particular distribution, while only having samples generated from a different distribution</mark>
- The equation for importance sampling is typically written as:$$\hat{\mu}_{q}=\frac{1}{N}\sum_{x_{i}\sim q(x)}\frac{p(x_{i})}{q(x_{i})}f(x_{i})\approx\mathbb{E}_{p(x)}[f(x)]$$
- Here, $\hat{\mu}_{q}$ is the estimated expected value of a function $f(x)$ under the probability distribution $p(x)$, using samples ${x_i}$ from a different distribution $q(x)$.
- The term $\frac{p(x_i)}{q(x_i)}$ is known as the <mark style="background: #FFB86CA6;">importance weight</mark>, and it corrects for the fact that the samples are drawn from $q(x)$ instead of $p(x)$.
- Disadvantages:
	- <mark style="background: #FFB86CA6;">High variance if p and q are very different</mark> (the proposal distribution $q(x)$ is not a good approximation of the target distribution $p(x)$)
	- Basically only one sample gets all the weight
![[Pasted image 20240403091135.png#invert|300]]
- The expectation of a constant is incorrectly estimated due to the weights
#### Self-normalized Importance sampling
- This failure case <mark style="background: #FFB86CA6;">can be fixed by normalizing the importance weights</mark>
- This is done by dividing each weight by the sum of all weights, which <mark style="background: #FFB86CA6;">ensures that the weights sum up to one</mark>$$\mathbb{E}_{p(x)}[f(x)] \approx\frac{1}{Z}\sum_{x_{i}\sim q(x)}\frac{p(x_{i})}{q(x_{i})}f(x_{i})$$, with $$Z=\sum_{i} \frac{p(x_{i})}{q(x_{i})}$$
- Advantages:
	- With an <mark style="background: #FFB86CA6;">infinite number of samples</mark>, the self-normalized estimator <mark style="background: #FFB86CA6;">will converge to the true expectation</mark> under the target distribution $p(x)$
	- Normalizing the weights <mark style="background: #FFB86CA6;">typically results in less variance compared</mark> to standard importance sampling because it <mark style="background: #FFB86CA6;">mitigates the effect of any single sample having an outsized impact on the estimate</mark>
- Disadvantages:
	- <mark style="background: #FF5582A6;">Introduces bias</mark>, particularly with a finite number of samples. This is because the <mark style="background: #FF5582A6;">normalization process itself alters the distribution of the weights</mark>, which can lead to an estimate that is systematically off from the true value

## Policy Gradient Algorithm
![[Pasted image 20240403091906.png#invert|]]
### Implicit Policy Gradient Objective
This objective seeks to adjust the policy parameters $\theta$ in a way that maximizes the expected return of actions taken under the policy. The gradient for updating the policy parameters is given by a specific formula involving a reweighting of samples based on their advantage, which is computed as the product of the probability ratio and the advantage estimate:

$$ \nabla_\theta J(\theta) \approx \frac{1}{N} \sum_{i=1}^{N} \sum_{t=1}^{T} \frac{\pi_\theta(a_{i,t} | s_{i,t})}{\pi_{\text{old}}(a_{i,t} | s_{i,t})} \nabla_\theta \log \pi_\theta(a_{i,t} | s_{i,t}) \hat{A}^{\pi_{\text{old}}}(s_{i,t}, a_{i,t}) $$
#### Components:
- **$\pi_\theta(a|s)$**: The policy parameterized by $\theta$, which determines the probability of taking action $a$ in state $s$.
- **$\pi_{\text{old}}(a|s)$**: The old policy, under which the data was collected.
- **$\hat{A}^{\pi_{\text{old}}}$**: The advantage function under the old policy, estimating how much better or worse taking action $a$ in state $s$ is compared to the average.
- **Probability Ratio**: $\frac{\pi_\theta(a|s)}{\pi_{\text{old}}(a|s)}$ <mark style="background: #FFB86CA6;">adjusts for the change in policy</mark>, focusing updates on improving decisions that are better than what the old policy suggested.
#### Comparison to Supervised Learning
In supervised learning, specifically in the context of maximum likelihood estimation (often used in imitation learning), the goal is to maximize the likelihood of observed actions given the states:

$$ \nabla_\theta \text{LogLik}(\theta) = \sum_{a_i, s_i} \nabla_\theta \log \pi_\theta(a_i|s_i) $$
This approach assumes a static dataset, where actions $a_i$ taken in states $s_i$ are treated as labels provided by a demonstrator (optimal or near-optimal behavior). The model (policy) is trained to replicate these actions.
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
#### Solution
The main goal is to maximize a policy's expected reward while keeping its deviation from a baseline policy within a defined limit, measured by the Kullback-Leibler (KL) divergence. This constraint ensures that the new policy does not stray too far from what has previously been established as reasonably effective, thus stabilizing the learning process.
- The problem is formulated as:
$$ \arg\max_{\pi} \sum_a \pi(a) r(a) \quad \text{s.t.} \quad KL(\pi \| q) \leq \epsilon, $$where $\pi(a)$ is the policy being optimized, $r(a)$ is the reward for action $a$, $q(a)$ is the baseline or old policy, and $\epsilon$ is a small positive number defining the trust region.
- The Lagrangian for this constrained optimization problem is:
$$ L(\pi, \eta, \lambda) = \sum_a \pi(a) r(a) + \eta \left( \epsilon - \sum_a \pi(a) \log \frac{\pi(a)}{q(a)} \right) + \lambda \left( \sum_a \pi(a) - 1 \right), $$
where $\eta$ is the Lagrange multiplier for the KL divergence constraint, and $\lambda$ enforces the probability distribution constraint $\sum_a \pi(a) = 1$.

##### Primal Solution
Taking the derivative of the Lagrangian with respect to $\pi(a)$ and setting it to zero gives:
$$ \frac{\partial L}{\partial \pi(a)} = r(a) - \eta \left( \log \frac{\pi(a)}{q(a)} + 1 \right) + \lambda = 0. $$
Solving for $\pi(a)$ gives:
$$ \pi(a) = q(a) \exp \left( \frac{r(a)}{\eta} - 1 \right) \exp \left( -\frac{\lambda}{\eta} \right). $$
This can be simplified to:
$$ \pi(a) = q(a) \exp \left( \frac{r(a) - \lambda + \eta}{\eta} \right). $$

##### Normalization
To ensure that $\pi$ is a valid probability distribution (i.e., sums to 1), normalize $\pi(a)$ across all actions:
$$ \pi(a) = \frac{q(a) \exp \left( \frac{r(a)}{\eta} \right)}{\sum_b q(b) \exp \left( \frac{r(b)}{\eta} \right)}. $$

##### Dual Problem
The dual of this optimization problem involves finding the optimal $\eta$ that maximizes the dual function:
$$ \max_{\eta > 0} \eta \epsilon + \eta \log \left( \sum_a q(a) \exp \left( \frac{r(a)}{\eta} \right) \right). $$
This dual problem is typically solved using numerical optimization techniques as it might not have a closed-form solution.
#### Problem
- Only valid for discrete actions and no states
- How can we do this for parametric policies such as Neural Networks?
	1. [[Natural Gradients]] and Trust-Region Policy Optimization ([[Trust-Region Policy Optimization (TRPO)|TRPO]])
	2. [[Proximal Policy Optimization (PPO)]]
	3. [[Differential Trust Region Layers]]