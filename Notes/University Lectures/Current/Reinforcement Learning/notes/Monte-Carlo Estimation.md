> [!summary] Definition
> Monte-Carlo Estimation allows to approximate an expectation by sampling:$$\mathop{\mathbb{E}}_{p(x)}[f(x)]=\int p(x)f(x)dx\approx \frac{1}{N}\sum_{x_{i} \sim p(x)}p(x_{i})$$ 

^d70ea3

- It states that the expected value of a function $f(x)$ under a probability distribution $p(x)$  can be approximated by averaging the values of $f(x)$ over a number of samples.
- This method involves using sample trajectories to estimate the value function, which represents the expected future reward if an agent is in a state and follows a policy.
## First Visit Monte Carlo
This method estimates the value of a state by averaging the returns following the first visits to that state over multiple episodes. 
![[Pasted image 20240708104420.png#invert|600]]
### 1. **Tracking First Visits:**
   - In this method, only the first time a state is visited during an episode is considered for the value estimation. If a state is visited multiple times in a single episode, only the first visit is used.
### 2. **Calculating Returns:**
   - The return for each first visit to a state $s$ is calculated as the total discounted reward received from that point until the end of the episode. This is represented by $R_t(s)$ in the image.
### 3. **Updating Value Estimates:**
   - The value of state $s$ is estimated by averaging these returns across multiple episodes. For instance, in the image, the state $s$ is visited four times in different episodes, with returns of +2, +1, -5, and +4. The estimated value $V^\pi(s)$ is calculated as $(2 + 1 - 5 + 4) / 4 = 0.5$.
## Constant Alpha Monte Carlo
This is a variant of the basic Monte Carlo method that uses a constant step-size parameter, $\alpha$, to compute a weighted average rather than a simple average:

$$ V^\pi(s_t) \leftarrow (1 - \alpha) V^\pi(s_t) + \alpha R_t $$

- **$\alpha$ (Learning Rate):** Determines how much the new information affects the existing estimate. A higher $\alpha$ gives more weight to recent returns, which can help in non-stationary environments or early in learning when the initial estimates may be poor.
- **$R_t$:** Is the return from the first visit to state $s$ during an episode.
## Limitations of Monte Carlo Methods
The description also highlights some limitations of Monte Carlo methods:
1. **Very Noisy Returns:**
   - **Stochasticity:** <mark style="background: #FFB86CA6;">Returns are influenced by the stochastic nature of the policy and the environment's dynamics</mark>, where each decision and outcome can introduce variability.
   - **Accumulated Noise:** Since the return is the sum of many steps rewards, <mark style="background: #FFB86CA6;">the noise from each step accumulates</mark>, increasing the overall variance of the return estimates.
2. **Sample Inefficiency:**
   - Monte Carlo methods can be sample inefficient because they <mark style="background: #FFB86CA6;">require a large number of episodes to converge</mark> to accurate estimates, particularly in environments with high variability in returns.
3. **Episode Completion Requirement:**
   - These methods <mark style="background: #FFB86CA6;">require the completion of entire episodes to make updates</mark>. This can be a drawback in very long or continuous tasks where the ends of episodes are not naturally defined or are very far apart.
