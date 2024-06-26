> [!summary] Definition
> [[Off-Policy]] actor-critic algorithms are a class of reinforcement learning methods that separate the policy used to interact with the environment (the behavior policy) from the policy being optimized (the target policy). This separation allows the algorithm to learn from experiences collected from a different policy than the one it’s currently optimizing, which can lead to more efficient learning. 

## Actor-Critic Framework
- The ‘actor’ $\pi_{\theta}(a|s)$ refers to the policy that decides which action to take
	- Stochastic actor update: Traditional methods 
	- Deterministic actor update: DDPG, TD3 
	- Variational actor update: SAC
- The ‘critic’ $Q_{\beta}(s,a)$ estimates the value function, which evaluates the potential future rewards of those actions.
	- Here, $Q_\beta(s, a)$ represents the critic’s estimate of the Q-value for state $s$ and action $a$ with $\beta$ denoting the parameters of the critic network.
### Updating the critic
- For the current policy, using off-policy data:$$\beta\leftarrow\beta+\alpha\sum_{i=1}^{N}(r_{i}+\gamma\mathbb{E}_{a'\sim\pi(\cdot|s'_{i})}\left[Q_{\beta'}(s_{t}',a')\right]-Q_\beta(s_{i},a_{i}))\frac{d}{dQ}Q_{\beta}(s_{i},a_{i})$$
- $\mathbb{E}_{a’ \sim \pi(\cdot|s’_{i})}$: The expected value over actions $a'$according to the target policy $\pi$ given the next state $s’_{i}$
- $Q_{\beta’}(s’_{i}, a’)$: The estimated value of the next state-action pair under the target policy, using the updated critic parameters $\beta'$
- $Q_\beta(s_{i}, a_{i})$: The estimated value of the current state-action pair using the current critic parameters $\beta$.
- $\frac{\partial}{\partial Q} Q_{\beta}(s_{i}, a_{i})$: The gradient of the Q-function with respect to its parameters $\beta$, evaluated at the current state-action pair.

- The expectation can be approximated using a single or multiple samples:$$\beta\leftarrow\beta+\alpha\sum_{i=1}^{N}(r_{i}+\gamma \frac{1}{K}\sum_{a_{i,k}'\sim\pi(\cdot|s_{i}')} Q_{\beta'}(s_{t}',a_{i,k}')-Q_\beta(s_{i},a_{i}))\frac{d}{dQ}Q_{\beta}(s_{i},a_{i})$$, typically $K=1$

The update rule works as follows:
1. For each sample $i$, calculate the temporal difference (TD) error, which is the difference between the estimated value of the current state-action pair and the sum of the received reward plus the discounted estimated value of the next state-action pair.
2. Multiply this TD error by the gradient of the Q-function with respect to its parameters. This product gives the direction in which the parameters should be adjusted to reduce the TD error.
3. Sum these products across all samples.
4. Update the critic parameters $\beta$ by taking a step in the direction that reduces the TD error, scaled by the learning rate $\alpha$.
### Updating the actor
- This updates the policy used by the algorithm
- Depends on the algorithm used
#### Stochastic Actor updates
- In traditional stochastic policy methods, the actor is updated using policy gradients.
- The update aims to increase the probability of actions that lead to higher returns.
- $$J_{PG}(\theta)=\mathbb{E}_{(s,a)\sim\pi(a|s)}\left[\hat A^{\pi_{old}}(s,a) \right]=\mathbb{E}_{(s,a)\sim\pi(a|s)}\left[Q_\beta(s,a)-V^{\pi_{old}}(s) \right]$$, then the gradient is given by:$$\begin{align} \nabla_{\theta} J_{PG}(\theta)=\mathbb{E}_{(s,a)\sim\pi(a|s)}\left[\nabla_{\theta}\log\pi_\theta(a|s_{i})( Q_\beta(s,a)-V^{\pi_{old}}(s)) \right] \\ \approx \frac{1}{N}\sum_{s_{i}}^{N}\mathbb{E}_{a\sim\pi(\cdot|s_{i})} \left[\nabla_{\theta}\log\pi_\theta(a|s_{i})( Q_\beta(s,a)-V^{\pi_{old}}(s)) \right]\end{align} $$
- <mark style="background: #FF5582A6;">But</mark>: Similar issues than standard [[Policy Optimization#Policy Gradient Methods|PG]] algorithms: Exploration might collapse too quickly 
	- In addition:
		- Likelihood ratio gradient makes sense if we assume that the objective (i.e. rewards) can be evaluated, but the analytical form (i.e. gradients) are unknown. 
		- However, in our case here we know (and its gradients)
		- Can this knowledge be exploited? 
			- Yes: Deterministic Policies: DDPG, Stochastic Policies: SAC
#### Deterministic Actor updates: Deep Deterministic Policy Gradients (DDPG)
- The main idea behind DDPG is to learn a deterministic policy $\pi(s)$ that can approximate the action which maximizes the Q-value for any given state $s$: $$\pi(s)\approx\arg\max_{a}Q_\beta(s,a)$$
- With deterministic policies the actor objective is given by: $$J_{DDPG}(\theta)=\mathbb{E}_{s\sim\mu^{\pi}(s)}\left[Q_\beta(s,\pi_\theta(s)) \right]$$
	- The expectation $\mathbb{E}_{s \sim \mu^{\pi}(s)}$ is taken over the state distribution $\mu^{\pi}(s)$ under the current policy $\pi$
- The gradients can now be computed by the chain rule:$$\begin{align} \nabla_\theta J_{DDPG}(\theta)=\mathbb{E}_{s\sim\mu^{\pi}(s)}\left[\nabla_{\theta}Q_\beta(s,\pi_\theta(s)) \right] \\ = \mathbb{E}_{s\sim\mu^{\pi}(s)}\left[\frac{\partial Q_\beta}{\partial a}(s,a=\pi(s))\frac{\partial\pi}{\partial\theta}(s) \right] \end{align}$$
	- The first term $\frac{\partial Q_\beta}{\partial a}(s, a = \pi(s))$ is the gradient of the Q-value with respect to the action, evaluated at the action chosen by the current policy. The second term $\frac{\partial \pi}{\partial \theta}(s)$ is the gradient of the policy with respect to its parameters.
- We can now use the gradient information $\frac{\partial Q}{\partial a}$ to do the actor update
	- -> the parameters of the actor network are updated in the direction that increases the expected Q-value

- Additional Tricks: 
	1. Add noise for exploration
		- The variance of the noise is typically kept constant and not adapted over time, although some implementations might decrease the noise as learning progresses.
	2. [[Value-Function Approximation#Q-Learning with Replay Buffers|Replay Buffers]] to get [[Off-Policy]]:
		- DDPG is an off-policy algorithm, meaning it can learn from experiences collected from a different policy than the one it’s currently optimizing.
		- A replay buffer is used to store experiences (state, action, reward, next state) collected from the environment.
	3. Increase stability for the Q-Function targets:
		- To stabilize the learning process, DDPG uses [[Value-Function Approximation#Target Networks|target networks]] for both the actor and the critic
		- The target networks are updated using [[Value-Function Approximation#Alternative Target Networks|Polyak averaging]]
![[Pasted image 20240407090308.png#invert|700]]
- Advantages:
	- Very sample efficient
- Disadvantages:
	- Overestimation Bias: fixed by [[Off-Policy Actor-Critic Algorithms#Deterministic actor update TD3|TD3]]
	- Poor estimation strategy: fixed by [[Off-Policy Actor-Critic Algorithms#Variational actor update SAC|SAC]]
#### Deterministic actor update: Twin-Delayed DDPG (TD3)
![[Value-Function Approximation#Overestimation Bias in Q-Values]]
- Even though TD3 does not use a max operation, there is still a bias due to the optimization process.
- The parameters optimized with the approximated Q-Function $\theta_{approx}$ are likely to overestimate the Q-values compared to the parameters optimized with the real Q-Function $\theta_{true}$ 
- Why:
	- Consider the parameters optimized with the approximated Q-Function$$\theta_{approx}=\theta+\alpha\mathbb{E}_{s\sim \mu^{\pi}(s)}\left[\frac{\partial Q_\beta}{\partial a}(s,a=\pi(s)) \frac{\partial\pi}{\partial\theta}(s) \right]$$
	- and the parameters optimized with the real Q-Function$$\theta_{true}=\theta+\alpha\mathbb{E}_{s\sim \mu^{\pi}(s)}\left[\frac{\partial Q^\pi}{\partial a}(s,a=\pi(s)) \frac{\partial\pi}{\partial\theta}(s) \right]$$
	- As gradient ascent is a local maximizer, we have with high probability: $$\begin{split} \mathbb{E}\left[Q_{\beta}(s,\pi_{approx}(s)) \right] \ge \mathbb{E}\left[Q_{\beta}(s,\pi_{true}(s)) \right] \\ \mathbb{E}\left[Q^{\pi}(s,\pi_{true}(s)) \right] \ge \mathbb{E}\left[Q^\pi(s,\pi_{approx}(s)) \right] \end{split}$$
		- Assuming that both optimizations achieved approximately a similar expected Q-value on their objectives, i.e.:$$\mathbb{E}\left[Q_{\beta}(s,\pi_{approx}(s)) \right] \approx \mathbb{E}\left[Q^{\pi}(s,\pi_{true}(s)) \right]$$
		- then: $$\mathbb{E}\left[Q_{\beta}(s,\pi_{approx}(s)) \right] \ge \mathbb{E}\left[Q^{\pi}(s,\pi_{approx}(s)) \right]$$
		- -> <mark style="background: #FF5582A6;">we overestimate</mark>, because gradient ascent will tend to find a local maximum for the approximated Q-function, which is not necessarily the true maximum for the real Q-function.

- TD3 addresses this issue by using a Clipped [[Value-Function Approximation#Double Q-Learning|Double Q-Learning]] algorithm:
	- Maintain 2 critics with parameters $\beta_{1},\ \beta_{2}$
	- Train 2 policies with parameters $\theta_{1}\ \theta_{2}$ (in the actual approach only $\theta'$ for computational efficiency)
	- Use $\theta_{1}$ to get targets for Q-Function $\beta_{2}$ and vice versa$$y_{1}=r_{i} +\gamma Q_{\beta_{1}'}(s_{i}',\pi_{\theta_{2}'}(s_{i}')),\ y_{2}=r_{i} +\gamma Q_{\beta_{2}'}(s_{i}',\pi_{\theta_{1}'}(s_{i}'))$$
		- -> No Optimization bias as $\pi_{\theta_{i}'}$ has been optimized w.r.t. a different objective
		- But: this is only true of both Q-functions are not correlated
	- Easy fix: Choose more pessimistic estimate for target values $$y_{i}=r_{i} +\gamma \min_{i=1,2} Q_{\beta_{i}'}(s_{i}',\pi_{\theta'}(s_{i}'))$$
		- -> We now get a pessimistic bias, but less severe as the pessimistic values do not propagate over time due to the optimization
		- Also only uses a single policy for computational efficiency
![[Pasted image 20240407093435.png#invert|700]]
- Advantages:
	- Very sample efficient
- Disadvantages:
	- Computational overhead due to 2 Q-Functions
#### Variational actor update: Soft-Actor Critic (SAC)
- So far, we still only looked at deterministic policies: 
	- Sub-optimal exploration strategy 
	- Hard to set hyper-parameters 
	- Can get stuck in local minima 
	- Can only learn one solution
- Solution: Keep diversity of the policy high! -> [[Maximum Entropy Reinforcement Learning]]
- 