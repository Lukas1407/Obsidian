> [!summary] Definition
>  The value function estimates the expected [[Markov Decision Process#Return|return]] of being in a particular state, or of performing a specific action in a given state.

There are two main types of value functions:
- [[Value-Function#State-Value Function/V-Function|V-Function]]
- [[Value-Function#Action-Value Function/Q-Function|Q-Function]]

## State-Value Function/V-Function
- $$V^{\pi}(\textbf{s})=\mathop{\mathbb{E}}[\sum_{t=0}^{\infty}\gamma^{t}r(s_{t},a_{t})|s_{0}=\textbf{s}]$$
- Predicts the expected [[Markov Decision Process#Return|return]] starting from state $\textbf{s}$ and then following a particular [[Policy|policy]]
- It answers the question, “What is the expected reward if I start in this state and behave optimally thereafter?”
- -> Quality measure of the state $\textbf{s}$
- $$V^{\pi}(s)=\sum_{a}\pi(a|s)Q^{\pi}(s,a)$$
- The optimal state-value function is given as: $$V^{*}(\textbf{s})=\max\limits_{\pi}V^{\pi}(\textbf{s})$$
### Bellman Equation 
$$V^{\pi}(\textbf{s})=\sum_{a}\pi(a|s)\left(r(s,a)+\gamma\sum_{s'}p(s'|s,a)V^{\pi}(s') \right)$$
- The Bellman equation expresses the value of a state as the sum of the immediate reward from the current state and the expected value of the subsequent state. This recursive nature reflects the idea that the value of a state is based on the rewards that can be obtained from that state onwards, under a certain policy.
- The future part of the value is an expectation because it accounts for the probabilistic nature of the transitions between states. It considers all possible next states, weighted by their transition probabilities.
- Optimality: In the context of optimal value functions, the Bellman equation reflects the best possible outcome from any given state, considering both immediate and future rewards.
#### Bellman Optimality Principle for the V-Function
- The Bellman Optimality Equation for the V-function (value function) can be derived from the general goal of maximizing the expected return (the total sum of discounted rewards) from any given state under the optimal policy. The equation is expressed as:

$$ V^*(s) = \max_a \left[ r(s, a) + \gamma \sum_{s'} p(s'|s, a) V^*(s') \right] $$
**Explanation:**
- **$V^*(s)$**: The optimal value function at state $s$, representing the maximum expected return starting from $s$ and following the best policy thereafter.
- **$r(s, a)$**: The immediate reward received after taking action $a$ in state $s$.
- **$\gamma$**: The discount factor, which accounts for the diminishing value of future rewards.
- **$p(s'|s, a)$**: The probability of transitioning to state $s'$ from state $s$ after action $a$ is taken.
- **$\max_a$**: The maximization over all possible actions from state $s$, reflecting the choice of the optimal action.
## Action-Value Function/Q-Function
- $$Q^{\pi}(\textbf{s},\textbf{a})=\mathop{\mathbb{E}}[\sum_{t=0}^{\infty}\gamma^{t}r(s_{t},a_{t})|s_{0}=\textbf{s}, a_{0}=\textbf{a}]$$
- Estimates the expected return after taking an action $\textbf{a}$ in state $\textbf{s}$ and then following a policy
- It answers the question, “What is the expected reward if I take this action in this state and then continue optimally?”
- -> Quality measure of state-action pair
- $$Q^{\pi}(s,a)= r(s,a)+\gamma\sum_{s'}p(s'|s,a)V^{\pi}(s')$$
### Bellman Equation 
$$Q^{\pi}(\textbf{s}, a)=r(s,a) +\gamma \sum_{s'}p(s'|s,a)\sum_{a'}\pi(a'|s')Q^{\pi}(\textbf{s}, a) $$
- The first part of the equation, $r(s,a) +\gamma \sum_{s'}p(s'|s,a)$, calculates the expected immediate reward and the transition to the next state $s'$ after taking action $a$ in state $s$
- The second part, $\sum_{a'}\pi(a'|s')Q^{\pi}(\textbf{s}, a)$, accounts for the discounted future rewards. It sums over all possible actions $a’$ in the next state $s’$, weighted by the probability of taking each action under policy $\pi$ and the Q-value of the resulting state-action pair.
#### Bellman Optimality Principle for the Q-Function
Similarly, the Bellman Optimality Equation for the Q-function (action-value function) is used to find the optimal action-value function, which gives the expected return of taking an action $a$ in state $s$ and thereafter following an optimal policy:

$$ Q^*(s, a) = r(s, a) + \gamma \sum_{s'} p(s'|s, a) \max_{a'} Q^*(s', a') $$
**Explanation:**
- **$Q^*(s, a)$**: The optimal Q-value for taking action $a$ in state $s$.
- **$\max_{a'}$**: The maximization over all actions $a'$ from the subsequent state $s'$, which indicates the choice of the best possible action after moving to state $s'$.
## Calculating Optimal Values
- The two basic approaches to compute the optimal action-values or state-values:
	 1. [[Value Iteration]]:
		- Every iteration updates both the values and (implicitly) the policy
		- We don’t track the policy, but taking the max over actions implicitly recomputes it 
		- Extreme case of policy iteration where we stop policy evaluation after one update
	 2. [[Policy Iteration]]
		 - We do several passes that update the value function with fixed policy
		 - After the policy is evaluated, a new policy is chosen (slow like a value iteration pass) 
		 - The new policy will be better (or we’re done)
 - Both algorithms compute a sequence of functions $Q_{k}(k=0,1,2,...)$ that converge to $Q^{*}$ or a sequence of functions $V_{k}(k=0,1,2,...)$ that converge to $V^*$
 - Both are dynamic programs for solving MDPs
### Does this work?
Yes but only in these 2 cases:
1. Discrete Systems: Policy and value iteration work well when the system is discrete, meaning there are a finite number of states and actions. This allows for exact calculations of the value functions and policies.
2. Linear Systems with Quadratic Rewards and Gaussian Noise (LQR): The reward function is quadratic and the noise is Gaussian. This is because the optimal control laws for LQR are well-understood and can be computed analytically.
In other cases, where the system is not discrete or does not fit the LQR framework, approximations are necessary because:
- Continuous State Spaces: Representing the value function ( V ) becomes challenging in continuous spaces, as it requires infinite granularity.
- Continuous Action Spaces: Solving the Bellman equations for continuous actions is difficult without discretization.
- Arbitrary Functions and Models: When the value function ( V ) and the model of the environment are complex, finding exact solutions is often infeasible.
Therefore, policy and value iteration are limited to scenarios where the environment’s dynamics are sufficiently simple to allow for exact computation. In more complex scenarios, approximation methods must be employed.
#### Solutions
- Sampling based approximations. which learn the value function from sample transitions $\tau=(s_{0},a_{0},r_{0},s_{1},a_{1},r_{1},...)$ without requiring a model of the environment $P(s'|s,a)$
- There are 2 methods:
	1.  [[Monte-Carlo Estimation]]
	2. [[Temporal Difference Learning]]