![[Pasted image 20240309133004.png#invert|]]
You can choose between k actions (arms) 
- Each arm returns a (stochastic) reward 
- The reward distributions $p(r|a)$ for each arm are unknown
Goal: Find the action that gives the most expected reward
- We need to know the expected reward per action -> the [[Value-Function#Action-Value Function/Q-Function|action-value function]]

## Action-Value Function
- We have no states, the q-Function is thereby: $$Q(\textbf{a})=\mathop{\mathbb{E}}[r_{t}|a_{t}=\textbf{a}] = \int p(r|\textbf{a})rdr$$
- In simpler terms, this equation tells us that the value of taking a certain action is the average of all possible rewards we could receive, each weighted by the likelihood of that reward occurring.
### Estimating Action-Values with Monte-Carlo Estimation
- ![[Monte-Carlo Estimation#^d70ea3]]
- The action-values can be approximated by:$$q^{*}(a)=\mathop{\mathbb{E}}[r_{t}|a]\approx \frac{\sum_{i=1}^{t-1}\mathop{\mathbb{I}}(a_{i}=a)r_{i}} {\sum_{i=1}^{t-1}\mathop{\mathbb{I}}(a_{i}=a)}:=q_{t}(a)$$
	- We sum up the reward each time the action $a$ is taken and divide it by the times $a$ was taken
	- -> we get an average value, which is our best estimate of the true action-value $q^*(a)$ based on our experiences up to time $t$ 

## Non-Stationary Bandit Problem
- The expected rewards of each arm in a k-armed bandit problem are subject to change over time
- The non-stationary aspect introduces additional complexity, as it requires strategies that can adapt to the changing reward probabilities to maximize the expected reward
### Using a constant step size
$$q_{n+1}=q_{n}+\alpha_{n}(r_{n}-q_{n}),$$ with $\alpha_{n}=\alpha$
- -> Older rewards from earlier times steps have less influence on the estimation $q_{n+1}$, as:$$q_{n+1}=(1-\alpha)^{n}q_{1}+\sum_{t=1}^{n}\alpha(1-\alpha)^{n-t}r_{t}$$
- This equation tells us that each reward $r_t$ is weighted by a factor of $\alpha(1-\alpha)^{n-t}$. Here’s why old rewards have less influence on the estimation:
	- **Decaying Influence**: The term $(1-\alpha)^{n-t}$ is a decay factor that reduces the influence of reward $r_t$ as the number of steps $n$ increases. Since $0 < \alpha < 1$ , the factor $(1-\alpha)$ raised to an increasing power $n-t$ becomes smaller for older rewards (where $t$  is small and $n-t$ is large).
	- **Immediate Rewards**: For more recent rewards, where $t$ is close to $n$, the term $n-t$ is small, and thus $(1-\alpha)^{n-t}$ is closer to 1, giving recent rewards a stronger weight in the estimation.
	- **Initial Value**: The initial estimate $q_1$ is also discounted by $(1-\alpha)^{n}$, which diminishes its impact over time as more rewards are observed.
- Often referred to as “moving” or “dynamic” averaging
