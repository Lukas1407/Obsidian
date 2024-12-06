> [!abstract] Definition
> Traditional methods like [[Trust-Region Policy Optimization (TRPO)]] and [[Proximal Policy Optimization (PPO)]] use approximations to enforce trust regions, which <mark style="background: #FFB86CA6;">can sometimes violate constraints or fail to find the optimal solution within the trust region</mark>.
Differentiable Trust Region Layers propose a solution to this by <mark style="background: #FFB86CA6;">introducing neural network layers that can enforce trust regions</mark> through closed-form projections.

- SOTA performance, no hacks needed

The original objective is: ![[Policy Optimization#^d61aef]]
If we can construct a policy $\textcolor{orange}{\tilde\pi_{\theta}(a|s)}$ that always satisfies the constraint:$$\mathcal{D}(\textcolor{orange}{\tilde\pi_{\theta}(a|s)},\pi_{old}(a|s))\le\epsilon,\ \forall s\in S$$
then the optimization reduces to:$$\theta_{new}=\arg\max_{\theta}\mathbb{E}_{(s,a)\sim\pi_{old}(a|s)}\left[\frac{\textcolor{orange}{\tilde\pi_{\theta}(a|s)}}{\pi_{old}(a|s)}\hat A^{\pi}(s,a)\right]$$
<mark style="background: #FFB86CA6;">For Gaussian Policies, we can split the constraints into mean and variance constraints</mark>:$$\begin{split} \mathcal{D}_{mean}(\textcolor{orange}{\tilde\pi_{\theta}(a|s)},\pi_{old}(a|s))\le\epsilon_{\mu},\ \forall s\in S \\ \mathcal{D}_{cov}(\textcolor{orange}{\tilde\pi_{\theta}(a|s)},\pi_{old}(a|s))\le\epsilon_{\Sigma},\ \forall s\in S\end{split}$$
- **Mean Constraint**: <mark style="background: #FFB86CA6;">Ensures that the changes in the policy's mean action values are within the bounds</mark> defined by $\epsilon_{\mu}$​.
	- The <mark style="background: #FFB86CA6;">mean of a Gaussian policy directly dictates the most likely action to be taken</mark> in a given state. It is central to the policy's performance, as it determines the expected action output.
- **Covariance Constraint**: Ensures that <mark style="background: #FFB86CA6;">changes in the policy's action distribution (variance) are within the bounds</mark> defined by $\epsilon_{\Sigma}$​.
	- The <mark style="background: #FFB86CA6;">covariance controls the exploration behavior of the policy by defining the spread around the mean action</mark>. A larger covariance implies more exploration, as the actions are more spread out around the mean.
- Mean and covariance <mark style="background: #FFB86CA6;">typically require different step-sizes</mark>
	- By using different step-sizes for the mean and covariance, each parameter can be updated in a manner that best suits its influence on the policy and its learning dynamics.
	- Adjusting the covariance with a different step-size helps maintain a stable balance between exploration (trying new actions to discover potentially better strategies) and exploitation (using known strategies to maximize reward).
	- Separate step-sizes allow for more efficient parameter updates.
- -> <mark style="background: #FFB86CA6;">Can achieve faster convergence</mark>

![[Pasted image 20240407075259.png#invert|700]]

### Mean Projection
**Objective**:
- The goal of mean projection is to <mark style="background: #FFB86CA6;">adjust the policy's mean action predictions such that they stay close to the previous policy's mean while also staying within a predefined trust region</mark>. This helps ensure the new policy does not deviate too drastically from what has previously been proven effective.

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
- Covariance projection aims to <mark style="background: #FFB86CA6;">adjust the policy's action distribution's spread or variability (covariance) to ensure it does not change too abruptly from one policy</mark> update to the next.

**Optimization Formulation**:
- Similar to the mean, the covariance optimization seeks to minimize the divergence (e.g., KL divergence) between the new and old covariance matrices while staying within a trust region defined by $\epsilon_{\Sigma}$.

**Closed-Form Solution**:
- For the KL divergence constraint, the closed-form solution for the new covariance $\tilde{\Sigma}$ is:
  $$
  \tilde{\Sigma}^{-1} = \frac{\Sigma^{-1}_{\theta} + \eta \Sigma^{-1}_{\text{old}}}{1 + \eta}
  $$
  where $\eta$ is another Lagrange multiplier. This solution indicates that the new inverse covariance is a weighted average of the new and old inverse covariances, again ensuring a controlled interpolation between updates.

## Properties of Differentiable Trust Region Layers:
- **Differentiability**: <mark style="background: #FFB86CA6;">Both mean and covariance projections</mark> are formulated in ways that allow gradients to be computed, enabling their integration into end-to-end trainable models.
- **Efficiency and Stability**: These methods allow for <mark style="background: #FFB86CA6;">more stable updates</mark> by controlling how much the policy parameters can change, hence reducing the risk of destabilizing the learning process.
- **Backpropagation**: The <mark style="background: #FFB86CA6;">closed-form solutions and the preservation of differentiability</mark> ensure that these projections can be seamlessly incorporated into standard deep learning architectures and optimized using conventional techniques like stochastic gradient descent.

