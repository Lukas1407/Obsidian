> [!summary] Definition
> In mathematics, a Markov decision process (MDP) provides a mathematical framework for modeling decision making in situations where outcomes are partly random and partly under the control of a decision maker.

## Definition
A MDP is a 4-tuple $(S,A,p,r)$:
- $S$: the possible states
- $A$: the possible actions
- $p(s'|s,a)$: the transition model telling how likely a next state $s'$ is given the current state $s$ and action $a$
- $r(s,a)$: the reward when being in state $s$ and choosing action $a$
- $\mu_{0}(s)$: the distribution over start stats
- Maybe a terminal state
![[Pasted image 20240309122455.png#invert|400]]
### Markov Property
- The action outcomes only depend on the current state, not on history
- $$p(S_{t+1}=s'|S_{t}=s_{t},A_{t}=a_{t},...,S_{0}=s_{0},A_{0}=a_{0})=p(S_{t+1}=s'|S_{t}=s_{t},A_{t}=a_{t})$$

## Optimization objective
- Find a good [[Policy]] $\pi(a|s)$ that specifies the action the decision maker has to choose in a given state
![[Pasted image 20240309122404.png#invert|300]]
- The objective is to choose a [[Policy]] $\pi$ that will maximize some cumulative function of the random rewards, typically the expected discounted sum over a potentially infinite horizon:$$\mathop{\mathbb{E}}[\sum_{t=0}^{\infty}\gamma^{t}r(s_{t},a_{t})],$$ where we choose $a_{t}=\pi(s_{t})$
### Discount factor $\gamma$
- $0 \le \gamma \le 1$
- The greater $\gamma$, the less the weight of the reward decreases with time -> optimizing long-term reward for $\gamma \to 1$
- The lower $\gamma$, the more the weight of the reward decreases with time -> optimizing short-term reward for $\gamma \to 0$
	- For $\gamma=0$, only the next time step will be taken into account
### Return
- The return $R$ is the discounted reward over all times steps
- $$R=\sum_{t=0}^{\infty}\gamma^{t}r(s_{t},a_{t})$$ 
### Infinite Reward?
- Problem: What if the game lasts forever? Do we get infinite rewards? 
- Solutions:
	 1. Finite horizon (also called episodic): (similar to depth-limited search): Terminate episodes after a fixed T steps; gives non-stationary policies ($\pi$ depends on time left)
	 2.  Discounting: use 0 < $\gamma$ < 1; Smaller $\gamma$ means smaller “horizon” – shorter term focus
	 3. Absorbing state: guarantee that for every policy, a terminal state will eventually be reached (like “overheated” for racing)

## MDP Search Trees
- When we know everything, e.i. all state transitions and rewards, we can solve a MDP as a search tree
![[Untitled.png#invert|600]]
- Problem:
	- Computationally inefficient due to the repetition of states and the potentially infinite horizon of the MDP
	- No reuse of computation, tree has to be computed from every state
- -> Therefore, dynamic programming algorithms like [[Policy Iteration]] and [[Value Iteration]] are used to solve MDPs more efficiently.