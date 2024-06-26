> [!abstract] Definition
>  A policy $\pi(a|s)$ defines the behavior of the RL agent, in that it specifies the action $a$ the agent will take in state $s$.

### Deterministic Policy
- $\pi(s)=a$
- If we have a deterministic environment -> $p(s,a)=s'$
- No randomness
### Stochastic Policy
- $\pi(a|s)$
- If we have a stochastic environment where we have a distribution over possible next states -> $p(s'|s,a)$