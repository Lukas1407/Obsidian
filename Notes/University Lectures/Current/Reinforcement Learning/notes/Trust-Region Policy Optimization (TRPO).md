> [!abstract] 
> Trust Region Policy Optimization (TRPO) is a reinforcement learning method designed to optimize an agent’s policy in a stable and efficient manner. The core idea behind TRPO is to <mark style="background: #FFB86CA6;">improve the policy by making small, incremental updates that are guaranteed not to decrease performance</mark>. This is achieved by defining a “[[Policy Optimization#Trust Regions|trust region]]” around the current policy and ensuring that updates do not deviate too far from it, which helps in maintaining stability and preventing drastic performance drops. 

- Advantages:
	- <mark style="background: #FFB86CA6;">First Policy Gradient Method that worked reliable for DNNs</mark>
	- Comes with <mark style="background: #FFB86CA6;">monotonic improvement guarantees</mark> (almost)
- Disadvantages:
	- Quite <mark style="background: #FFB86CA6;">complicated to implement</mark>
	- Still <mark style="background: #FFB86CA6;">rather slow</mark> (computationally - does not scale to large networks) due to FIM
	- Rarely used nowadays
## Equation
$$\begin{split} \arg\max_{\theta} \frac{1}{N}\sum_{i}\sum_{t}\frac{\pi_\theta(a_{i,t}|s_{i,t})}{\pi_{old}(a_{i,t}|s_{i,t})}\hat A^{\pi_{old}}(s_{i,t},a_{i,t}) \\ s.t. \ \mathbb{E}_{\mu^{\pi_{old}(s)}}{[KL(\pi_{\theta}(a|s)\parallel\pi_{old}(a|s))]}\le\epsilon \end{split}$$
- $\arg \max_\theta$: This term seeks the value of theta that maximizes the following expression.
- $1/N \sum_i \sum_t$: This is the average over all $N$ sampled trajectories, where $i$ indexes the trajectory and $t$ indexes the time step within each trajectory.
- $\pi_\theta(a_{i,t}|s_{i,t})/\pi_{old}(a_{i,t}|s_{i,t})$: This is the probability ratio of the new policy $\pi_\theta$ to the old policy $\pi_{old}$ for taking action $a_{i,t}$ in state $s_{i,t}$.
- $\hat A^pi_{old}(s_{i,t},a_{i,t})$: This is the [[Policy Optimization#^bc0da1|advantage function]] under the old policy, indicating the relative quality of action $a_{i,t}$ in state $s_{i,t}$.
- $\mathbb{E}_{\mu^\pi_{old}}$: This denotes the expected value under the state distribution $\mu$ induced by the old policy $\pi_{old}$.
- $KL(\pi_{\theta}(a|s) parallel \pi_{old}(a|s))$: This is the [[Kullback-Leiber Divergence]], which measures the difference between the new policy $\pi_\theta$ and the old policy $\pi_{old}$.
- $\le \epsilon$: This indicates that the expected KL divergence should be less than a small threshold epsilon, which defines the trust region’s size.

- **Intuition Behind the Equation:** The TRPO algorithm seeks to improve the policy by maximizing the expected advantage while ensuring that the new policy does not deviate too much from the old policy. The constraint with the [[Kullback-Leiber Divergence|KL divergence]] ensures that the updates to the policy are kept within a trust region, preventing large, potentially destabilizing updates. This balance allows for consistent improvement in the policy’s performance while maintaining stability during training.