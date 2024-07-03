> [!summary] 
>  The idea is to start by assuming that all actions are very rewarding until proven otherwise. By initializing the estimated values of actions (Q-values) to high optimistic values, the agent is encouraged to try out all actions.

- Since the agent starts with high expectations for all actions, it will explore different actions to correct these estimates
- Optimistic value initialization naturally balances exploration and exploitation without the need for additional exploration mechanisms like epsilon-greedy or Boltzmann policies. Initially, the agent explores due to high Q-values, but as it learns, exploration decreases, and exploitation increases.