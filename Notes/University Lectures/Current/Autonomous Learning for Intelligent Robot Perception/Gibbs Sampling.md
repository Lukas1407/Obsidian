1. **Process Overview**:
   - Gibbs sampling generates samples by iteratively updating each variable conditioned on the current values of all other variables.
   - Starting with an initial set of values $\{z_i : i = 1, \dots, M\}$, the algorithm iteratively samples each variable $z_i$ from its **conditional distribution**.
2. **Sampling Steps**:
   - For each time step $\tau = 1, \dots, T$:
     - Sample $z_1^{(\tau+1)} \sim p(z_1 | z_2^{(\tau)}, \dots, z_M^{(\tau)})$
     - Sample $z_2^{(\tau+1)} \sim p(z_2 | z_1^{(\tau+1)}, z_3^{(\tau)}, \dots, z_M^{(\tau)})$
     - Continue this process for all $M$ variables.
3. **Key Idea**:
   - Each update draws from the **full conditional** distribution of a variable given the current values of the other variables.
   - This is useful in **graphical models** like Markov Random Fields, where we can utilize the **Markov blanket** (set of neighboring nodes) to determine conditionals.

## Gibbs Sampling as a Special Case of Metropolis-Hastings
1. **Proposal Distribution**:
   - In Gibbs sampling, the proposal distribution for each variable $x_i$ is the **conditional distribution** $p(x_i' | x_{\neg i})$, where $x_{\neg i}$ represents all other variables.
2. **Acceptance Rate**:
   - Since the proposal distribution is the conditional distribution, the acceptance rate $\alpha$ is always 1:
     $$
     \alpha = \frac{p(x') q(x | x')}{p(x) q(x' | x)} = 1
     $$
   - This means every proposed sample is accepted, simplifying the sampling process.
3. **Limitations**:
   - Although every proposal is accepted, Gibbs sampling may not converge faster because it only updates one variable at a time, which can result in slow mixing, especially in high-dimensional spaces.
## Example of Gibbs Sampling on a Binary Image (Ising Model)
![[Untitled 3.png#invert|300]]
1. **Ising Model Setup**:
   - An example application of Gibbs sampling is on a **Markov Random Field (MRF)** representing a binary image.
   - **Edge Potentials** $\psi(x_s, x_t) = \exp(J x_s x_t)$: Control the interaction between neighboring pixels, encouraging similar values (smooth regions).
   - **Node Potentials** $\psi(x_t) = \mathcal{N}(y_t | x_t, \sigma^2)$: Model the likelihood of each pixel value based on observed data.
2. **Sampling Process**:
   - Each pixel $x_t$ is sampled based on its conditional probability given its neighbors, iteratively updating all pixels.
   - The progression shows the binary image (letter "A") becoming more defined after each sampling round.
## Application to Gaussian Mixture Models (GMM)
![[Pasted image 20241111081842.png#invert|300]]
1. **Mixture of Gaussians**:
   - Gibbs sampling can be applied to **Gaussian Mixture Models (GMMs)**, where the goal is to sample data points and parameters from a mixture of Gaussian distributions.
   - Left: Contour plot of three Gaussian components with different mixing coefficients.
   - Right: Samples generated from the GMM, showing the distribution of points among the Gaussian components.
2. **Importance of Gibbs Sampling in GMMs**:
   - Each component in the GMM has a mixing coefficient, and Gibbs sampling allows us to update assignments of data points to components iteratively based on current parameters.
### General Formulation of Gaussian Mixture Models
1. **Full Joint Distribution**:
   - The GMM’s joint distribution includes:
     - Data likelihood: Gaussian for each data point.
     - Correspondence probability: Multinomial distribution over the component assignments.
     - Mixture prior: Dirichlet distribution for the mixing coefficients.
     - Parameter priors: For mean and covariance.
2. **Model Parameters**:
   - $\mu$: Means of the Gaussian components.
   - $\Sigma$: Covariances of the Gaussian components.
   - $\pi$: Mixing proportions.
### Gibbs Sampling for GMMs
1. **Conditional Distributions**:
   - Gibbs sampling in GMMs starts with the full joint distribution and iteratively samples from the **full conditionals**:
     - $p(z_i = k | x_i, \mu, \Sigma, \pi)$: Probability of assigning data point $i$ to component $k$.
     - $p(\pi | z)$: Dirichlet distribution over mixing coefficients.
     - $p(\mu_k | \Sigma_k, Z, X)$: Normal distribution for the component mean.
     - $p(\Sigma_k | \mu_k, Z, X)$: Inverse-Wishart distribution for the component covariance.

2. **Iterative Process**:
   - Initialize all variables, then sample each variable conditioned on the others until convergence.
![[Pasted image 20241111081917.png#invert|400]]
### Convergence and Mixing Time
1. **Convergence**:
   - The graph shows that after about 50 sample rounds, the values stabilize, indicating convergence.
2. **Mixing Time**:
   - The **mixing time** $\tau_\epsilon$ indicates how long it takes for the chain to converge and depends on the **eigen gap** $\gamma = \lambda_1 - \lambda_2$ of the transition matrix:
     $$
     \tau_\epsilon \leq O\left( \frac{1}{\gamma} \log \frac{n}{\epsilon} \right)
     $$
   - The larger the eigen gap, the faster the convergence.
