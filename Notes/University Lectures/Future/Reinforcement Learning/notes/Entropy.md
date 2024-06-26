- Entropy $H(\pi)$ measures the uncertainty in the policy. It’s defined as:$$H(\pi) = E_\pi[-\log(\pi(a))] = -\sum_a\pi(a)\log(\pi(a))
$$
- A uniform distribution $\pi(a)=\frac{1}{|\mathcal{A}|}$ has the highest entropy, indicating maximum uncertainty or exploration, while a deterministic policy has zero entropy.