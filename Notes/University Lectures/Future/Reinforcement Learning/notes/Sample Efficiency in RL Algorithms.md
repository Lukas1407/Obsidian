![[Pasted image 20240309131909.png#invert|]]

## Why do we want to use less sample efficient algorithms?
### Stability and easy of use
- Does it converge?
- And if it converges, to what? 
- And does it converge every time?
- Why is this even a question?: 
	- In RL, the situation is more complex because the algorithms are not just minimizing a loss function; they are interacting with an environment and learning a policy for decision-making. 
	- This interaction can lead to more complex convergence behaviors.
- Examples
#### Value Function Fitting:
- At best, minimizes the Bellman error
- At worst, doesn’t optimize anything – <mark style="background: #FF5582A6;">Many popular deep RL value fitting algorithms are not guaranteed to converge to anything in the nonlinear case</mark>
#### Policy Gradient
- The only method that actually performs gradient descent (ascent) on the true objective
- But usually needs a hell lot of samples
### Scalability
### Quality of the final policy