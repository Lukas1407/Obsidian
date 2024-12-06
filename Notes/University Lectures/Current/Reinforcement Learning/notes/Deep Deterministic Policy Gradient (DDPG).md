The main idea behind DDPG is <mark style="background: #FFB86CA6;">to learn a deterministic policy</mark> $\pi(s)$ that <mark style="background: #FFB86CA6;">can approximate the action which maximizes the Q-value for any given state</mark> $s$: $$\pi(s)\approx\arg\max_{a}Q_\beta(s,a)$$
- With deterministic policies the actor <mark style="background: #FFB86CA6;">objective is given by</mark>: $$J_{DDPG}(\theta)=\mathbb{E}_{s\sim\mu^{\pi}(s)}\left[Q_\beta(s,\pi_\theta(s)) \right]$$
	- The expectation $\mathbb{E}_{s \sim \mu^{\pi}(s)}$ is taken over the state distribution $\mu^{\pi}(s)$ under the current policy $\pi$
- <mark style="background: #FFB86CA6;">The gradients can now be computed by the chain rule</mark>:$$\begin{align} \nabla_\theta J_{DDPG}(\theta)=\mathbb{E}_{s\sim\mu^{\pi}(s)}\left[\nabla_{\theta}Q_\beta(s,\pi_\theta(s)) \right] \\ = \mathbb{E}_{s\sim\mu^{\pi}(s)}\left[\frac{\partial Q_\beta}{\partial a}(s,a=\pi(s))\frac{\partial\pi}{\partial\theta}(s) \right] \end{align}$$
	- The first term $\frac{\partial Q_\beta}{\partial a}(s, a = \pi(s))$ is the gradient of the Q-value with respect to the action, evaluated at the action chosen by the current policy. The second term $\frac{\partial \pi}{\partial \theta}(s)$ is the gradient of the policy with respect to its parameters.
- We can now use the gradient information $\frac{\partial Q}{\partial a}$ to do the actor update
	- -> the parameters of the actor network are updated in the direction that increases the expected Q-value

- Additional Tricks: 
	1. <mark style="background: #FFB86CA6;">Add noise for exploration</mark>
		- The <mark style="background: #FFB86CA6;">variance of the noise is typically kept constant</mark> and not adapted over time, although some implementations might decrease the noise as learning progresses.
	2. [[Value-Function Approximation#Q-Learning with Replay Buffers|Replay Buffers]] to get [[Off-Policy]]:
		- DDPG is an off-policy algorithm, meaning it can learn from experiences collected from a different policy than the one it’s currently optimizing.
		- A replay buffer is used to store experiences (state, action, reward, next state) collected from the environment.
	3. <mark style="background: #FFB86CA6;">Increase stability for the Q-Function targets</mark>:
		- To stabilize the learning process, DDPG uses [[Value-Function Approximation#Target Networks|target networks]] for both the actor and the critic
		- The target networks are updated using [[Value-Function Approximation#Alternative Target Networks|Polyak averaging]]
![[Pasted image 20240407090308.png#invert|700]]
- Advantages:
	- <mark style="background: #FFB86CA6;">Very sample efficient</mark>
- Disadvantages:
	- <mark style="background: #FFB86CA6;">Overestimation Bias</mark>: fixed by [[Off-Policy Actor-Critic Algorithms#Deterministic actor update TD3|TD3]]
	- <mark style="background: #FFB86CA6;">Poor estimation strategy</mark>: fixed by [[Off-Policy Actor-Critic Algorithms#Variational actor update SAC|SAC]]