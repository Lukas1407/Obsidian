Even though TD3 does not use a max operation, there is still a bias due to the optimization process.
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