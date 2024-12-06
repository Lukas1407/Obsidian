> [!summary] Definition
> Proximal Policy Optimization (PPO) is a type of [[Policy Optimization#Policy Gradient Methods|policy gradient method]] for reinforcement learning which <mark style="background: #FFB86CA6;">aims to improve the stability and efficiency of the training process</mark>. It uses importance weighted objective with [[Policy Optimization#Generalized Advantage Estimation (GAE)|GAE]].

- The core idea behind PPO is to <mark style="background: #FFB86CA6;">prevent too large updates to the policy at each step</mark>, which helps in avoiding destructively large policy updates that might cause performance collapse.

- <mark style="background: #FFB86CA6;">Advantages</mark>:
	- Currently the SOTA for [[On-Policy]]
	- Easy to implement
	- Very impressive results
	- “Throw tons of GPUs on the problem” and it works
- Disadvantages:
	- Very hacky theory, <mark style="background: #FFB86CA6;">many hacks needed</mark> in addition
	- <mark style="background: #FFB86CA6;">Performance boost comes from hacks, not from objective</mark>
	- <mark style="background: #FFB86CA6;">Hyper-parameters can be difficult to tune</mark>
	- Does not work well for harder exploration problems
## PPO v1: Dual Descent PPO
![[Pasted image 20240407072944.png#invert|700]]
- **Relaxed Constrained Optimization**: PPO v1 formulates the policy update problem as a constrained optimization problem where the <mark style="background: #FFB86CA6;">objective is to maximize the policy's expected return while keeping the KL divergence between the new and old policies below a threshold</mark> $\epsilon$.
- **Lagrangian and Dual Descent**: The problem uses a Lagrangian formulation involving a dual variable $\eta$ to balance the objective and the constraint. The optimization alternates between maximizing the Lagrangian w.r.t. policy parameters $\theta$ and minimizing it w.r.t. $\eta$, effectively performing dual descent.
## PPO v2: Clipped Policy Gradient Objective
![[Pasted image 20240407073526.png#invert|700]]
- **Clipped Objective**: <mark style="background: #FFB86CA6;">Rather than using a KL divergence constraint, PPO v2 introduces a clipping mechanism in the objective function that limits the ratio of new policy probabilities to old policy probabilities</mark>, denoted by $w_i(\theta) = \frac{\pi_\theta(a_i|s_i)}{\pi_{\text{old}}(a_i|s_i)}$. The clipping is defined by parameters $\epsilon$, typically set around 0.1 to 0.2.
     - **Objective Function**: The objective becomes:
       $$
       J_{\text{clip}}(\theta) = \sum_i \min \left(w_i(\theta) A_i, \text{clip}(w_i(\theta), 1-\epsilon, 1+\epsilon) A_i\right),
       $$
       where $A_i$ is the advantage at time-step $i$. This form ensures that the policy update is significant only when the change in policy probability remains within a specified range.
- The clipping mechanism prevents the policy from moving too far from the old policy, acting as a [[Policy Optimization#Trust Regions|trust region method]]

