> [!summary] Definition
The KL divergence is a commonly used measure of dissimilarity between two probability distributions.
>  $$KL(p(x)\parallel q(x))=\int p(x)\log \frac{p(x)}{q(x)}dx$$

It has the following properties:
- It is always non-negative.
- It is zero if and only if both distributions are identical.
- It is non-symmetric, meaning $KL(p \parallel q) \neq KL(q \parallel p)$ 

For Gaussian distributions, the KL divergence can be computed in closed form, which involves comparing:
- The shapes and orientations (rotations) of the distributions.
- The entropies of the distributions.
- The means of the distributions, where the metric space is defined by the inverse of the covariance matrix. A higher dissimilarity is indicated if there is less variance in the direction of the mean updates.