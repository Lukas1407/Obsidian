> [!summary] Definition
> Monte-Carlo Estimation allows to approximate an expectation by sampling:$$\mathop{\mathbb{E}}_{p(x)}[f(x)]\approx \frac{1}{N}\sum_{x_{i} \sim p(x)}p(x_{i})$$ 

^d70ea3

- It states that the expected value of a function $f(x)$ under a probability distribution $p(x)$  can be approximated by averaging the values of $f(x)$ over a number of samples.
- This method involves using sample trajectories to estimate the value function, which represents the expected future reward if an agent is in a state and follows a policy.

## First visit Monte Carlo
This method estimates the value function by averaging the returns following the first visit to each state. It only considers the first time a state is visited in an episode when calculating the average.
- Actual average of returns
## Constant-α Monte Carlo
Instead of averaging all returns, this method uses a constant step-size parameter, $α$, to update the value function. After each episode, the value function for the visited states is updated as a weighted average of the previous value and the new sample return, with $α$ determining the weight given to the new sample.
- ,Uses a weighted average, allowing it to adapt more quickly to changes because it gives more weight to recent returns.

## Limitations
Monte Carlo (MC) methods for estimating value functions have several limitations due to the nature of the returns they use for estimation:
- **Noisy Returns**: The returns can be very noisy because they are affected by the stochasticity (randomness) in the policy’s actions and the environment’s dynamics. Since the return is the sum of rewards over many steps, each step can introduce variability, leading to noisy estimates.
- **Sample Inefficiency**: MC methods can be sample inefficient because they require a large number of episodes to converge to an accurate estimate. This is because each estimate is based on complete episodes, and the variance in returns can be high, requiring more samples to average out the noise.
- **Delayed Updates**: MC methods must wait until the end of an episode to update the value function. This means that learning can only occur after an entire sequence of actions and rewards has been observed, which can slow down the learning process, especially in long or continuous tasks.

These limitations make MC methods less suitable for problems with long episodes, high variance in returns, or when quick updates to the policy are necessary. Alternative methods like Temporal Difference learning can address some of these issues by updating estimates based on incomplete episodes.