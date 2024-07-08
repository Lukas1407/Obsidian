The FIM $\mathcal{F}$ measures the amount of information that an observable random variable carries about unknown parameters upon which the probability depends. In the context of policy gradients:
- $\mathcal{F}$ is calculated as:
  $$
  \mathcal{F} = \mathbb{E}_{\mu^{\pi_{old}}(s)} \left[ \left( \frac{\partial \log \pi(a|s)}{\partial \theta} \right) \left( \frac{\partial \log \pi(a|s)}{\partial \theta} \right)^T \right],
  $$
  where $\mu^{\pi_{old}}(s)$ is the state distribution under the old policy, and $\pi(a|s)$ is the probability of taking action $a$ in state $s$ under the policy parameterized by $\theta$.
