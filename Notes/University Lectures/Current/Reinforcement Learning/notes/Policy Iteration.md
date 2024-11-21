> [!summary] Definition
>  Policy iteration is a method used in RL to find the optimal [[Policy]], which is the strategy that yields the maximum reward over time. It involves two main steps: policy evaluation and policy improvement, which are repeated iteratively until the policy converges to the optimal one.

## Algorithm
### 1. Initialization
- Start with an arbitrary [[Policy]] $\pi$, which is a mapping from states to actions.
### 2. Policy Evaluation
- Calculate the [[Value-Function#State-Value Function/V-Function|state-value function]] $V^\pi(s)$ for the current policy $\pi$. 
- This involves assessing how good it is to follow the current policy from each state, in terms of expected rewards.
![[Pasted image 20240310122152.png#invert|600]]
- Iterative use of the [[Value-Function#Bellmann Equation|bellmann equation]]
#### Sample-based Policy Evaluation
- Since the full model of the environment may not be available or the state space may be too large, the expectations in the policy evaluation can be approximated using sampled transition trajectory $\tau=(s_{0},a_{0},r_{0},s_{1},a_{1},r_{1}...)$ rather than summing over all possible transitions.
1. **Sampling an Action**: An action $a_t$ is sampled according to the policy's distribution $\pi(\cdot|s_t)$.
2. **Sampling the Next State**: The next state $s_{t+1}$ is sampled from the probability distribution $P(\cdot|s_t, a_t)$, which might typically be derived from interacting with the environment.
3. **Computing a New Target**: For the current time-step $t$, compute the target $y_t$ which is an estimate of the expected return from state $s_t$ given the sampled action and the next state:
   $$ y_t = r(s_t, a_t) + \gamma V^\pi(s_{t+1}) $$

- To incorporate the sampled estimate into the current value function estimate, a moving average approach is used:

$$ V^\pi(s_t) \leftarrow (1 - \alpha) V^\pi(s_t) + \alpha y_t $$

- 2 Options:
	- [[Temporal Difference Learning]]
	- [[Monte-Carlo Estimation]]
### 3. Policy Improvement
- Update the policy by making it greedy with respect to the current value function. This means for each state, choose the action that maximizes the expected return using the current value estimates.
- Can be done using the [[Value-Function#Action-Value Function/Q-Function|Q-Function]] or the [[Value-Function#State-Value Function/V-Function|V-Function]]:
	- $$\pi_{new}(s)=arg\max_{a}Q^{\pi}(s,a)$$
	- $$\pi_{new}(s)= arg\max_{a}\left(r(s,a)+\gamma\sum_{s'}p(s'|s,a)V_{k-q}^{\pi}(s')\right)$$
- It can be shown that $Q^{\pi_{new}}(s,a)\ge Q^{\pi}(s,a)$ -> the new policy performs strictly better (or equally good) than the old policy in every state
#### Stochastic Formulation of Policy Improvement
##### Using the V-Function
The policy improvement step using the V-function (Value function) is shown here:

$$ \pi_{\text{new}}(a|s) = \begin{cases} 
1, & \text{if } a = \arg\max_{a'} \left(r(s, a') + \gamma \sum_{s'} p(s'|s, a') V^{\pi}(s')\right) \\
0, & \text{else} 
\end{cases} $$

- The policy is updated such that the action which maximizes the expected return (considering immediate rewards and discounted future value of subsequent states) is chosen deterministically (the probability of choosing  is set to 1) for each state . All other actions have a probability of 0.
##### Using the Q-Function
The policy improvement step using the Q-function (Action-Value function) is shown here:

$$ \pi_{\text{new}}(a|s) = \begin{cases} 
1, & \text{if } a = \arg\max_{a'} Q^\pi(s, a') \\
0, & \text{else} 
\end{cases} $$
- **Policy Improvement**: The policy is similarly updated deterministically. The action $a'$ that maximizes the Q-value in state $s$ is chosen with a probability of 1. This simplifies the decision-making process as it directly uses the action-value function without needing to consider the transition probabilities and rewards separately as in the V-function case.
### 4. Iteration
- Repeat the policy evaluation and improvement steps until the policy no longer changes, indicating convergence.
## Summary
The policy iteration algorithm can be summarized by the following steps:
![[Pasted image 20240310123530.png#invert|600]]

## Convergence 
-  It is guaranteed to converge to the optimal policy and value function for a finite [[Markov Decision Process|MDP]], as there is only a finite number of policies.
- Max-Norm: The max-norm is a way to measure the size of vectors, defined as the maximum absolute value of its elements. It’s represented as:$$\|U\|_{\infty} = \max_s |U(s)|
$$
- This norm is used to measure the difference between two vectors (or functions) in an infinite-dimensional space.

- Theorem 1: This theorem states that for any two approximations $U$ and $V$, the difference between them will decrease with each iteration of the algorithm, provided the algorithm is a contraction mapping. The equation is:$$\|U^{\pi}_{k+1} - V^{\pi}_{k+1}\|_{\infty} \leq \gamma \|U^{\pi}_{k} - V^{\pi}_{k}\|_{\infty}
,$$where $\gamma$ is a discount factor less than 1. This implies that the approximations are converging towards each other, and eventually towards the true value function $V’$.

- Theorem 2: The second theorem provides a condition for convergence. If the change in our approximation is small enough, it suggests that we are close to the true value function $V'$. The equation is:$$\|V^{\pi}_{k+1} - V^{*}\|_{\infty} \leq \epsilon \Rightarrow \|V^{\pi*} - V^{*}\|_{\infty} \leq \frac{2\epsilon\gamma}{1-\gamma}
,$$where $\epsilon$ is a small positive number, $V^{\pi*}$ is the optimal value function. This theorem assures us that if our iterative updates result in only small changes, we are close to finding the true optimal values.

These concepts are crucial in understanding how iterative algorithms can be used to solve problems where the goal is to find an optimal solution that maximizes or minimizes a certain objective, such as in Markov Decision Processes (MDPs) in reinforcement learning. The slide emphasizes the importance of convergence in iterative methods and how it can be assured mathematically.