1. **Steps**:
   - **Initialization**: Start with an initial state $x^0$.
   - **Iteration**:
     - Propose a new state $x' \sim q(x' | x^s)$ from the current state $x^s$.
     - Compute the **acceptance probability** $\alpha$:
       $$
       \alpha = \frac{\tilde{p}(x') q(x | x')}{\tilde{p}(x) q(x' | x)}
       $$
     - Generate a random number $u$ from a uniform distribution $U(0,1)$.
     - Accept $x'$ as the new state $x^{s+1}$ if $u < \alpha$; otherwise, stay at $x^s$.
2. **Purpose**:
   - The algorithm creates a sequence of samples that, over time, approximates the target distribution, even if $\tilde{p}$ is not normalized.
## Why Metropolis-Hastings Works
1. **Detailed Balance**:
   - To ensure that the chain converges to the target distribution, we need to show that it satisfies the **detailed balance** condition:
     $$
     p(x) p_{\text{MH}}(x' | x) = p(x') p_{\text{MH}}(x | x')
     $$
   - Here, $p_{\text{MH}}(x' | x)$ is the transition probability under the Metropolis-Hastings algorithm.
2. **Proof**:
   - The transition probability can be split into the proposal probability and the acceptance probability.
   - Detailed balance is satisfied by the construction of the acceptance probability, ensuring that the chain has the target distribution as its stationary distribution.
3. **Validity**:
   - This formulation works for both **discrete** and **continuous** variables.
## Choosing the Proposal Distribution
1. **Requirements**:
   - The **proposal distribution** $q(x' | x)$ must allow non-zero probability of moving to any state that has a non-zero probability in the target distribution.
   - Common choices include **Gaussian** proposals, especially in continuous spaces, as they cover all states with non-zero probability.
2. **Variance of the Proposal**:
   - **Low Variance**: If the variance of the Gaussian proposal is too low, the sampler may not explore the distribution well, potentially getting stuck in one mode.
   - **High Variance**: If the variance is too high, many proposed moves are likely to be rejected, leading to inefficiency and slow convergence.
### Example - Gaussian Mixture Target
![[University Lectures/Current/Autonomous Learning for Intelligent Robot Perception/images/Untitled 2.png#invert|300]]
1. **Target**:
   - The target distribution is a **mixture of two 1D Gaussians**.
2. **Proposal Distribution**:
   - The proposal distribution is a Gaussian with varying variances in different scenarios.
3. **Effect of Variance on Sampling**:
   - This example illustrates the impact of choosing different variances for the proposal distribution:
     - **Low Variance**: The sampler does not move far from its current state, which may result in poor exploration.
     - **High Variance**: The sampler proposes moves that are too far and are often rejected.
   - The **optimal variance** allows the sampler to explore the target distribution effectively without too many rejections.
### Summary
The **Metropolis-Hastings algorithm** is a powerful method for MCMC sampling. It uses a proposal distribution to generate new candidate states and an acceptance probability to decide whether to move to the proposed state. The algorithm’s success hinges on satisfying **detailed balance**, which ensures that the chain will converge to the target distribution.

- **Key Components**:
  - **Proposal Distribution** $q(x' | x)$: Controls the exploration of the space.
  - **Acceptance Probability**: Ensures that the chain converges to the target distribution by adjusting for differences between the proposal and target distributions.

- **Choosing the Proposal Variance**: 
  - Selecting an appropriate variance for the proposal is crucial for effective sampling. Too low or too high variance can lead to inefficient exploration or high rejection rates.

The Metropolis-Hastings algorithm is widely used in scenarios where direct sampling is difficult, allowing efficient exploration of complex, high-dimensional distributions.




