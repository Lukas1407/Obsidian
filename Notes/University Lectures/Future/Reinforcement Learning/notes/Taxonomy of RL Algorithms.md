1. [[Value-Function|Value-based]] such as [[Value-Function Approximation#Deep Q-Learning Network (DQN)|DQN]]
	1. <mark style="background: #BBFABBA6;">✓</mark> Rather sample-efficient (allows off-policy)
	2. <mark style="background: #BBFABBA6;">✓</mark> Yield state-of-the-art performance in many domains 
	3. <mark style="background: #FF5582A6;">×</mark> No convergence guarantees × Often quite hard to tune... 
	4. <mark style="background: #FF5582A6;">×</mark> Hard to use for continuous action spaces 
	5. <mark style="background: #FF5582A6;">×</mark> Approximation errors in the Q-function might bias the quality of the resulting policy
2. [[Policy Optimization]]
	1. <mark style="background: #BBFABBA6;">✓</mark> Easier to use and tune 
	2. <mark style="background: #BBFABBA6;">✓</mark> More compatible with rich architectures (including recurrence) 
	3. <mark style="background: #BBFABBA6;">✓</mark> More versatile, more compatible with auxiliary objectives 
	4. <mark style="background: #BBFABBA6;">✓</mark> (Almost) no bias -> finds good solutions 
	5. <mark style="background: #FF5582A6;">×</mark> Needs (much) more samples
3. Actor-Critic

## Why so many methods?
### Different trade-offs:
1. Sample efficiency:
	- How many samples do we need to get a good policy?
	- Mostly determined by the fact if the policy is [[On-Policy]] or [[Off-Policy]]
2. Stability & ease of use 
- Different assumptions:
	- Stochastic or deterministic? 
	- Continuous or discrete? 
	- Episodic or infinite horizon? 
- Different things are easy or hard in different settings
	- Easier to represent the policy?
	- Easier to represent the model?