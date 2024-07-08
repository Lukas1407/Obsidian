- **Uncertainty in Estimates**: Since the true action values are unknown, we rely on estimates based on limited data. However, we can calculate [[Confidence Bounds]] for these estimates to understand the potential range of the true action values.
> [!summary] Idea
>  The strategy suggests being optimistic about actions with high potential, even if their estimated value is not the highest. This approach encourages exploration of actions that might be undervalued.

- The brackets show the [[Confidence Bounds]] 
![[Pasted image 20240310103612.png#invert|600]]

## UCB Action Selection
$$a_t = \operatorname*{argmax} \limits_{a} \left[ q(a) + c\sqrt{\frac{\log t}{N_t(a)}} \right],
$$ with: 
- $a_t$: The action selected at time $t$.
- $q(a)$: The estimated value of action $a$, which represents exploitation.
- $c$: A constant that scales the level of exploration.
- $\log t$: The natural logarithm of the current time step $t$, which increases over time.
- $N_t(a)$: The number of times action $a$ has been selected up to time $t$
### Intuition
- The term $q(a)$ encourages the selection of actions that have high estimated values, focusing on exploitation.
- The second term, $c\sqrt{\frac{\log t}{N_t(a)}}$, represents the uncertainty or variance in the estimate of action $a$. It becomes larger for actions that have been selected less often, encouraging exploration.
	- Exploration diminishes!! Unlike [[Epsilon-Greedy Action Selection]], where the exploration rate is fixed 
- By adding these two terms, UCB selects actions that not only have high estimated rewards but also accounts for the uncertainty in those estimates, thus balancing the need to explore less certain actions with the desire to exploit actions with high estimated rewards.
### Example
$$c\sqrt{\frac{\log 10000}{5000}} = 0.042c$$
$$c\sqrt{\frac{\log 10000}{100}} = 0.303c$$
-> The less number of times action a has been chosen, the higher the bonus!
- The bonus is inversely proportional to $N_t(a)$

- This relates to [[Optimistic Value Initialization|optimistic initialization]] of the Q-values in the following way:
	- Optimistic Initialization: By setting the initial Q-values to be high (optimistic), all actions are considered to be potentially rewarding at the start. This encourages the agent to explore different actions.
	- UCB Exploration Bonus: Similarly, the UCB bonus acts as a dynamic form of optimistic initialization. Actions with less certainty (chosen less often) are given a higher bonus, which is like temporarily increasing their Q-values, thus encouraging the agent to explore them.
	- Both methods serve to promote exploration, but while optimistic initialization is a static method applied at the beginning of learning, the UCB bonus is a dynamic method that adjusts the level of exploration throughout the learning process based on the frequency of each action being chosen.