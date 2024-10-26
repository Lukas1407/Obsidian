In **offline** Bayesian linear regression, we aim to find the **Maximum A Posteriori (MAP)** estimate of the parameters $w$ given all data points $X$ and their targets $y$.

1. **Likelihood**:
   - The likelihood of observing $y$ given $X$ and $w$ is:
     $$
     p(y | X, w, \sigma_l) = \prod_{i=1}^N \mathcal{N}(y_i; \phi(x_i)^T w, \sigma_l^2)
     $$
   - Here, $\phi(x_i)$ represents a basis function applied to $x_i$, and $\sigma_l^2$ is the observation noise variance.

2. **Prior on $w$**:
   - A Gaussian prior on the model parameters $w$:
     $$
     p(w | \sigma_p) = \mathcal{N}(w; 0, \sigma_p^2 I)
     $$
   - This prior assumes that our initial belief about $w$ is centered around zero with covariance $\sigma_p^2 I$.

3. **MAP Estimate**:
   - The MAP approach finds the parameters $w$ that maximize the posterior distribution $p(w | X, y)$, effectively combining information from the prior and the likelihood.