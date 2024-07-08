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
![[Deep Deterministic Policy Gradient (DDPG)]]
#### Deterministic actor update: Twin-Delayed DDPG (TD3)
![[Value-Function Approximation#Overestimation Bias in Q-Values]]
![[Twin-Delayed DDPG (TD3)]]
#### Variational actor update: Soft-Actor Critic (SAC)
- So far, we still only looked at deterministic policies: 
	- Sub-optimal exploration strategy 
	- Hard to set hyper-parameters 
	- Can get stuck in local minima 
	- Can only learn one solution
- Solution: Keep diversity of the policy high! -> [[Maximum Entropy Reinforcement Learning]]
![[Soft-Actor Critic (SAC)]]