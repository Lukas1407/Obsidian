> [!summary] Definition
>  Off-policy learning allows the agent to learn from actions that are outside the current [[Policy]], such as using a secondary [[Policy]] for learning while following a different policy for making decisions.
>  This approach enables the agent to learn from experiences collected previously, under different policies.

- **Off-Policy**: Learn from others’ experiences, using data from different policies.

- Advantages:
	- Able to improve the policy without generating new samples from that policy (i.e. reuse experience from old policies): Since off-policy methods are not restricted to learning from the current policy, they can maintain and update a separate value function that is independent of the policy used to generate the data. This allows them to benefit from a larger pool of experiences without the need to collect new data after every policy update.
		- -> [[Sample Efficiency in RL Algorithms]]
	- Efficient data use
- Disadvantages:
	- Can be more complex