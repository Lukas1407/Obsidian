> [!abstract] Definition
> Reinforcement Learning (RL) is a type of machine learning where an agent learns to make decisions by interacting with an environment. 

- The core goal for an agent is to learn how to take a series of actions that lead to the best possible outcome.
- An RL agent improves its behavior over time by continuously interacting with its environment, making decisions, observing the outcomes, and learning from the results.
- In every RL scenario, there is some form of optimality criterion, often called a reward function. 
	- This function evaluates the actions taken by the agent and assigns a score or reward based on the desirability of the outcomes.
- RL differs from many other machine learning approaches in that the agent typically starts with little to no knowledge about the environment in which it operates. 
	- It <mark style="background: #FFB86CA6;">does not have a pre-programmed strategy</mark> or understanding of how its actions affect the environment. 
	- Instead, the agent must discover this through its own interactions, figuring out the dynamics and mechanics of the environment as it learns. 
	- -> This makes RL applicable to problems where the model of the environment is complex, unknown, or difficult to formulate explicitly.

Examples of RL:
- Beat the Backgammon Champion
- Grasping of Objects
-  Control of Power-Plants
- Management of a Portfolios

Basic reinforcement learning is modeled as a [[Markov Decision Process]].

## Why is RL so hard?
### Sequential Interaction
- **Temporal Credit Assignment**: 
	- Determining which actions are responsible for eventual outcomes 
	- When rewards are delayed, it becomes difficult to link a particular reward back to the specific actions that caused it, especially when those actions were taken several steps before receiving the reward.
- **Delayed Rewards**:
	- Unlike in supervised learning where the feedback to an action is immediate and clearly correct or incorrect, in RL, rewards can be delayed over many time steps.
- **Less immediate reward may yield higher long-term reward**
	- This point highlights the strategic challenge in RL. Sometimes, the best strategy involves forgoing an immediate reward for a greater long-term benefit. Learning to recognize and optimize for such long-term outcomes requires sophisticated strategies and poses a substantial challenge.
### We need to explore
- **[[Exploration-Exploitation Trade-off|Exploration vs. Exploitation Dilemma]]**: In RL, an agent must decide between exploring new actions to discover potentially better rewards (exploration) and leveraging known actions that already give good rewards (exploitation). Balancing these two is crucial as too much exploration can lead to inefficiency, and too much exploitation can prevent finding the optimal policy.
- **Sparse and Noisy Rewards**: Rewards in RL are often sparse (not provided at every time step, maybe only in the end) and noisy (may include random variations), which complicates the learning process. An agent must learn from limited feedback that may not always clearly indicate the best course of action.
### Partial Observability
- In many real-world applications, an agent cannot observe the entire state of the environment but must operate in situations of partial observability. For instance, an autonomous vehicle might only have access to current sensor readings (like images or LIDAR data) without full knowledge of the environment. This uncertainty adds complexity to decision-making processes.
### Non-Stationarity
- **Real-world environments change over time**: Many environments are non-stationary, meaning that the underlying dynamics of the environment can change independently of the agent's actions. This aspect requires the agent to continually adapt its strategy to maintain performance, unlike stationary problems often tackled in other areas of machine learning.
### High Dimensional Continuous States and Actions
- **High dimensional continuous states and actions**: The state and action spaces in many RL problems can be extremely large and continuous. Managing high-dimensional data requires complex function approximation techniques that can generalize well across a continuous range. This is technically challenging and computationally demanding.
## Index
1. [[Components of a RL Agent]]
2. [[Anatomy of RL Algorithms]]
3. [[Taxonomy of RL Algorithms]]
4. [[K-Armed Bandit Problem]]
5. [[Incremental Update Rule]]
6. [[Exploration-Exploitation Trade-off]]
7. [[Multi-step Return]]
8. [[Bias-Variance Trade-off]]
9. [[QT-Opt|Q-Learning with continuous actions]]
10. [[Off-Policy Actor-Critic Algorithms]]
11. [[Black-Box Methods]]
12. [[Motion Promitives (MPs)]]