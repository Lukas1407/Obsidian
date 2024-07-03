> [!summary] Definition
>  In on-policy learning, the agent learns the value of the [[Policy]] being used to make decisions, which is also the policy being improved upon.
>  This means the agent learns from the actions it takes based on its current policy.

- **On-Policy**: Learn on the job, using the policy you’re currently employing.

- Advantages:
	- Can be simpler and more stable
- Disadvantages:
	- Each time the [[Policy]] changes, we need to generate new samples: When the policy changes, even slightly, <mark style="background: #FFB86CA6;">the expected outcomes and rewards can also change</mark>. Therefore, <mark style="background: #FFB86CA6;">the agent needs to interact with the environment again</mark> to obtain new samples that accurately reflect the performance of the updated policy
		- -> [[Sample Efficiency in RL Algorithms]]
	- Require more data to converge