![[Pasted image 20240309131909.png#invert|800]]

## Why do we want to use less sample efficient algorithms?
### Stability and easy of use
- Sometimes, more sample-efficient algorithms are less stable or harder to tune and deploy. A less efficient algorithm might be preferred if it offers greater stability or ease of use.
#### Essential Properties
- **Convergence**: It’s crucial to know if an algorithm converges, meaning it will reach a stable solution.
- **Convergence to What?**: If an algorithm converges, the next question is what it converges to: the global optimum, a local optimum, or something else?
- **Consistency of Convergence**: Does the algorithm converge under all initial conditions and across all potential environments?
#### Why Are These Questions Relevant?
- **Supervised Learning**: In supervised scenarios, algorithms typically use gradient descent and converge to a local optimum. This process is well-understood and stable.
- **Reinforcement Learning**: In RL, the situation is more complex:
    - **Q-learning** uses fixed-point iteration, not optimizing directly for the expected reward but rather seeking a stable policy.
    - **Model-based RL** involves learning a model that may not directly optimize for expected reward.
    - **Policy Gradient Methods** do use a form of gradient ascent but are often sample-inefficient despite directly optimizing the expected reward.
#### Examples
##### Value Function Fitting:
- At best, minimizes the Bellman error
- At worst, doesn’t optimize anything – <mark style="background: #FF5582A6;">Many popular deep RL value fitting algorithms are not guaranteed to converge to anything in the nonlinear case</mark>
##### Policy Gradient
- The only method that actually performs gradient descent (ascent) on the true objective
- But usually needs a hell lot of samples
### Scalability
### Quality of the final policy