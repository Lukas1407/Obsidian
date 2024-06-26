
> [!summary] 
> This approach aims to develop a policy that not only seeks to maximize rewards but also maintains a level of randomness to explore potentially better actions, which is particularly useful in environments with uncertainty and changing conditions. The inclusion of entropy ensures that the policy doesn’t become too deterministic and is able to explore effectively. 

A more sophisticated explorations strategy than [[Epsilon-Greedy Action Selection]], as it incorporates entropy to balance exploration and exploitation.

![[Entropy]]

- Maximum Entropy Objective: This objective introduces a trade-off between exploitation (choosing actions with high ( q )-values) and exploration (ensuring high entropy)2. The equation for the Maximum Entropy Objective is:$$\pi^* = \text{argmax}_\pi \left( \sum_a \pi(a) \left( q(a) + \frac{H(\pi)}{\tau} \right) \right),$$where $\tau$ is a temperature parameter that controls the trade-off. A higher $\tau$ encourages more exploration, while a lower $\tau$ favors exploitation.

## Soft-Max/Boltzmann Policy
- The policy derived from the Maximum Entropy Objective is often referred to as the soft-max or Boltzmann policy and looks like this: $$\pi^*(a) = \frac{\exp(q(a))}{Z} = \frac{\exp(\tau q(a))}{\sum_{a'}\exp(\tau q(a'))}
$$
- **Boltzmann Distribution**: The Boltzmann policy uses a mathematical function from statistical mechanics called the Boltzmann distribution. This function assigns a probability to each action that is exponentially proportional to its estimated value. The formula is:$$\pi (a) = \frac {\frac{\exp(q(a))}{\tau}}{\sum_{b}\frac{\exp( q(b)}{\tau }}$$
- **Temperature Parameter**: The temperature parameter $\tau$ plays a crucial role in the Boltzmann policy. When $\tau$ is high, the policy tends towards equal probabilities for all actions (more exploration). When $\tau$ is low, the policy becomes more greedy, favoring the actions with the highest estimated values (more exploitation).
- **Balancing Act**: The beauty of the Boltzmann policy is that it provides a systematic way to balance between exploring new actions and exploiting known good actions. It ensures that while the best-known actions are chosen more frequently, there is still a chance to explore other actions, which could potentially lead to better outcomes.
### Using Neural Networks 
- Can also be defined using outputs of a neural network $p_{\theta}(s,a)$
$$\pi_{\theta}(a|s) \frac{\exp(p_{\theta}(s,a)}{\sum_{a'}\exp(p_{\theta}(s,a')}$$