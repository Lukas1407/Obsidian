> [!abstract] Definition
> Traditional methods like [[Trust-Region Policy Optimization (TRPO)]] and [[Proximal Policy Optimization (PPO)]] use approximations to enforce trust regions, which can sometimes violate constraints or fail to find the optimal solution within the trust region.
Differentiable Trust Region Layers propose a solution to this by <mark style="background: #FFB86CA6;">introducing neural network layers that can enforce trust regions</mark> through closed-form projections.

- SOTA performance, no hacks needed

The original objective is: ![[Policy Optimization#^d61aef]]
If we can construct a policy $\textcolor{orange}{\tilde\pi_{\theta}(a|s)}$ that always satisfies the constraint:$$\mathcal{D}(\textcolor{orange}{\tilde\pi_{\theta}(a|s)},\pi_{old}(a|s))\le\epsilon,\ \forall s\in S$$
then the optimization reduces to:$$\theta_{new}=\arg\max_{\theta}\mathbb{E}_{(s,a)\sim\pi_{old}(a|s)}\left[\frac{\textcolor{orange}{\tilde\pi_{\theta}(a|s)}}{\pi_{old}(a|s)}\hat A^{\pi}(s,a)\right]$$
For Gaussian Policies, we can split the constraints into mean and variance constraints:$$\begin{split} \mathcal{D}_{mean}(\textcolor{orange}{\tilde\pi_{\theta}(a|s)},\pi_{old}(a|s))\le\epsilon_{\mu},\ \forall s\in S \\ \mathcal{D}_{cov}(\textcolor{orange}{\tilde\pi_{\theta}(a|s)},\pi_{old}(a|s))\le\epsilon_{\Sigma},\ \forall s\in S\end{split}$$
- **Mean Constraint**: Ensures that the changes in the policy's mean action values are within the bounds defined by $\epsilon_{\mu}$​.
	- The mean of a Gaussian policy directly dictates the most likely action to be taken in a given state. It is central to the policy's performance, as it determines the expected action output.
- **Covariance Constraint**: Ensures that changes in the policy's action distribution (variance) are within the bounds defined by $\epsilon_{\Sigma}$​.
	- The covariance controls the exploration behavior of the policy by defining the variability or spread around the mean action. A larger covariance implies more exploration, as the actions are more spread out around the mean.
- Mean and covariance typically require different step-sizes
	- By using different step-sizes for the mean and covariance, each parameter can be updated in a manner that best suits its influence on the policy and its learning dynamics.
	- Adjusting the covariance with a different step-size helps maintain a stable balance between exploration (trying new actions to discover potentially better strategies) and exploitation (using known strategies to maximize reward).
	- Separate step-sizes allow for more efficient parameter updates.
- -> Can achieve faster convergence

![[Pasted image 20240407075259.png#invert|700]]

Mean projection and covariance projection are key components of Differentiable Trust Region Layers in reinforcement learning. They ensure that policy updates remain within a specified trust region, which is crucial for stable and effective learning. Here’s an explanation of both:
### Mean Projection
**Objective**:
- The goal of mean projection is to adjust the policy's mean action predictions such that they stay close to the previous policy's mean while also staying within a predefined trust region. This helps ensure the new policy does not deviate too drastically from what has previously been proven effective.

**Optimization Formulation**:
- The optimization problem is expressed as:
  $$
  \arg\min_{\tilde{\mu}} \|\tilde{\mu}(s) - \mu_{\theta}(s)\|^2_{\Sigma^{-1}_{\text{old}}}
  $$
  subject to a constraint on the Mahalanobis distance (which uses the old policy's covariance as the metric matrix) between the new mean $\tilde{\mu}$ and the old mean $\mu_{\text{old}}$ being less than $\epsilon_{\mu}$.
**Lagrangian Solution**:
- The solution involves a Lagrangian formulation where the Lagrange multiplier $\omega$ is adjusted to satisfy the constraint. The optimal mean $\tilde{\mu}$ is found as a weighted average between the new policy mean $\mu_{\theta}(s)$ and the old policy mean $\mu_{\text{old}}(s)$:
  $$
  \tilde{\mu} = \frac{\mu_{\theta}(s) + \omega \mu_{\text{old}}(s)}{1 + \omega}
  $$
  This represents a linear interpolation between the old and new means, controlled by the multiplier $\omega$, which is derived based on the trust region constraint.
### Covariance Projection
**Objective**:
- Covariance projection aims to adjust the policy's action distribution's spread or variability (covariance) to ensure it does not change too abruptly from one policy update to the next.

**Optimization Formulation**:
- Similar to the mean, the covariance optimization seeks to minimize the divergence (e.g., KL divergence) between the new and old covariance matrices while staying within a trust region defined by $\epsilon_{\Sigma}$.

**Closed-Form Solution**:
- For the KL divergence constraint, the closed-form solution for the new covariance $\tilde{\Sigma}$ is:
  $$
  \tilde{\Sigma}^{-1} = \frac{\Sigma^{-1}_{\theta} + \eta \Sigma^{-1}_{\text{old}}}{1 + \eta}
  $$
  where $\eta$ is another Lagrange multiplier. This solution indicates that the new inverse covariance is a weighted average of the new and old inverse covariances, again ensuring a controlled interpolation between updates.

**Properties of Differentiable Trust Region Layers**:
- **Differentiability**: Both mean and covariance projections are formulated in ways that allow gradients to be computed, enabling their integration into end-to-end trainable models.
- **Efficiency and Stability**: These methods allow for more stable updates by controlling how much the policy parameters can change, hence reducing the risk of destabilizing the learning process.
- **Backpropagation**: The closed-form solutions and the preservation of differentiability ensure that these projections can be seamlessly incorporated into standard deep learning architectures and optimized using conventional techniques like stochastic gradient descent.
### Conclusion
By implementing mean and covariance projections as differentiable trust region layers within a neural network, reinforcement learning models can maintain more stable and reliable convergence. This approach ensures that updates to the policy are significant enough to improve performance but not so large as to cause instability, providing a robust framework for continuous policy improvement.

