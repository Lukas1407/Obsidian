## Motivation for Variational Inference in Modeling Human Behavior:
**Limitations of Traditional RL and Optimal Control:**
- Traditional models typically assume deterministic and fully rational behavior, which may not align with the stochastic nature of human decisions. Classical approaches do not naturally account for the variability and unpredictability inherent in human behavior.
**Probabilistic Inference as a Model for Human Behavior:**
- Probabilistic models, as opposed to deterministic ones, offer a more nuanced understanding of behavior that incorporates uncertainty and the stochastic nature of decision-making. This makes them better suited for modeling real human actions.
**Reformulating RL and Control:**
- By integrating principles of probabilistic inference, reinforcement learning and optimal control models can be reformulated to better capture the stochasticity of human behavior. This perspective is crucial in fields like inverse reinforcement learning, where the goal is to discern underlying motivations or reward functions from observed behaviors.
**Implications:**
- Viewing RL and planning through the lens of probabilistic inference changes the algorithmic structure and offers new tools for algorithm design, making these methods more robust and more representative of actual human decision-making processes.
**Conclusion:**
- Understanding RL and optimal control through the lens of probabilistic inference not only aligns more closely with observed human behavior but also enhances the ability of these algorithms to operate effectively in complex, uncertain environments. This approach fosters a deeper integration of AI systems into real-world applications where human-like decision-making is crucial.

## Variational Inference Overview
### Key Concepts:
- **Target Distribution $p^*(x)$**: This is often an intractable distribution due to an unknown normalization constant $Z$, which makes direct computation and sampling difficult. The equation $p^*(x) = \frac{p^*(x)}{Z}$ shows how the actual probability distribution is normalized by $Z$.
- **Tractable Distribution $q(x)$**: Typically, a simpler distribution (like a Gaussian) that is used to approximate the intractable $p^*(x)$.
### Intractability:
- The distribution $p^*(x)$ is termed intractable due to the difficulty in evaluating the normalization constant $Z$ and challenges in sampling from $p^*(x)$ directly.
### Utility:
- Variational inference is useful for approximating posterior distributions in Bayesian inference, for latent variable models, and significantly, in the context of reinforcement learning where many models or environments may be complex or unknown.
### How can we do that?
-> We can minimize a divergence measure between both distributions
**Optimization Objective:**
- To approximate the target distribution $p^*(x)$, we aim to minimize a divergence measure between $q(x)$ and $p^*(x)$. This is usually the Kullback-Leibler (KL) divergence.
**Divergence Types:**
- **Reverse KL-Divergence $KL(q(x) \| p^*(x))$**: This form focuses on where $q(x)$ assigns probability relative to $p^*(x)$. It's particularly useful when we want $q(x)$ to be zero where $p^*(x)$ is small, thus avoiding assigning probabilities to unlikely events.
- **Forward KL-Divergence $KL(p^*(x) \| q(x))$**: This form forces $q(x)$ to cover areas where $p^*(x)$ is large, ensuring that all significant modes of $p^*(x)$ are represented in $q(x)$.

