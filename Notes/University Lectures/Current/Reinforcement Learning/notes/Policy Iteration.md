> [!summary] Definition
>  Policy iteration is a method used in RL to find the optimal [[Policy]], which is the strategy that yields the maximum reward over time. It involves two main steps: policy evaluation and policy improvement, which are repeated iteratively until the policy converges to the optimal one.

## Algorithm
### 1. Initialization
- Start with an arbitrary [[Policy]] $\pi$, which is a mapping from states to actions.
### 2. Policy Evaluation
- Calculate the [[Value-Function#State-Value Function/V-Function|state-value function]] $V^\pi(s)$ for the current policy $\pi$. This involves assessing how good it is to follow the current policy from each state, in terms of expected rewards.
![[Pasted image 20240310122152.png#invert|]]
- Iterative use of the [[Value-Function#Bellmann Equation|bellmann equation]]
### 3. Policy Improvement
- Update the policy by making it greedy with respect to the current value function. This means for each state, choose the action that maximizes the expected return using the current value estimates.
- Can be done using the [[Value-Function#Action-Value Function/Q-Function|Q-Function]] or the [[Value-Function#State-Value Function/V-Function|V-Function]]:
	- $$\pi_{new}(s)=arg\max_{a}Q^{\pi}(s,a)$$
	- $$\pi_{new}(s)= arg\max_{a}\left(r(s,a)+\gamma\sum_{s'}p(s'|s,a)V_{k-q}^{\pi}(s')\right)$$
### 4. Iteration
- Repeat the policy evaluation and improvement steps until the policy no longer changes, indicating convergence.

The policy iteration algorithm can be summarized by the following steps:
![[Pasted image 20240310123530.png#invert|]]

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