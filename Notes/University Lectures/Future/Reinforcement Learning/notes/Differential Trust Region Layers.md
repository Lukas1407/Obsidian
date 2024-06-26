> [!abstract] Definition
> Traditional methods like [[Trust Region Policy Optimization (TRPO)]] and [[Proximal Policy Optimization (PPO)]] use approximations to enforce trust regions, which can sometimes violate constraints or fail to find the optimal solution within the trust region.
Differentiable Trust Region Layers propose a solution to this by <mark style="background: #FFB86CA6;">introducing neural network layers that can enforce trust regions</mark> through closed-form projections.

- SOTA performance, no hacks needed

The original objective is: ![[Policy Optimization#^d61aef]]
If we can construct a policy $\textcolor{orange}{\tilde\pi_{\theta}(a|s)}$ that always satisfies the constraint:$$\mathcal{D}(\textcolor{orange}{\tilde\pi_{\theta}(a|s)},\pi_{old}(a|s))\le\epsilon,\ \forall s\in S$$
then the optimization reduces to:$$\theta_{new}=\arg\max_{\theta}\mathbb{E}_{(s,a)\sim\pi_{old}(a|s)}\left[\frac{\textcolor{orange}{\tilde\pi_{\theta}(a|s)}}{\pi_{old}(a|s)}\hat A^{\pi}(s,a)\right]$$
For Gaussian Policies, we can split the constraints into mean and variance constraints:$$\begin{split} \mathcal{D}_{mean}(\textcolor{orange}{\tilde\pi_{\theta}(a|s)},\pi_{old}(a|s))\le\epsilon_{\mu},\ \forall s\in S \\ \mathcal{D}_{cov}(\textcolor{orange}{\tilde\pi_{\theta}(a|s)},\pi_{old}(a|s))\le\epsilon_{\Sigma},\ \forall s\in S\end{split}$$
- Mean and covariance typically require different step-sizes
- -> Can achieve faster convergence

![[Pasted image 20240407075259.png#invert|700]]

Mean projection and covariance projection are key components of Differentiable Trust Region Layers in reinforcement learning. They ensure that policy updates remain within a specified trust region, which is crucial for stable and effective learning. Here’s an explanation of both:
## Mean Projection:
- The objective for mean projection is to minimize the distance between the new policy mean and the old policy mean while staying within the trust region.
- Mathematically, it’s formulated as an optimization problem where the goal is to find the projected mean (\tilde{\mu}) that is closest to the original mean (\mu) under the trust region constraints.
- The solution involves using Lagrangian multipliers to find the optimal $\tilde{\mu}$ that satisfies the trust region constraints:$$\tilde\mu=\frac{\mu_{\theta}(s)+\omega\mu_{old}(s)}{1+\omega}$$, $\omega$ is a Lagrange multiplier that is determined during the optimization process to satisfy the trust region constraint.
- Trust region mean is linear interpolation between new and old mean 
- Closed form solution for dual exists (no additional optimization needed) 
- We can fully back-propagate through this layer

## Covariance Projection:
- Covariance projection focuses on the policy’s covariance matrix, ensuring that the updated covariance does not deviate significantly from the previous one.
- Similar to mean projection, it’s an optimization problem where the goal is to find the projected covariance matrix (\tilde{\Sigma}) that is closest to the original (\Sigma), again subject to trust region constraints.
- The optimization is solved using differentiable techniques, allowing for efficient computation of the projected covariance matrix
- Closed form solution for [[Kullback-Leiber Divergence|KL]]:$$\tilde\Sigma^{-1}=\frac{\Sigma_{\theta}^{-1}+\eta\Sigma_{old}^{-1}}{1+\eta}$$
- Trust region precision is linear interpolation between new and old precision matrices
- needs to be found by convex optimization (CO) of the dual function 
- CO is still differentiable at its optimum due to the use of convex optimization layers