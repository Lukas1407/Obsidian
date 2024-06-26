![[Pasted image 20240312135847.png#invert|]]
  
Once the optimal value function ( V^_(s) ) is found, the optimal policy ( \pi^_(s) ) can be derived by choosing the action that maximizes the expected return from each state:
$$
π^∗(s)=\operatorname*{argmax} \limits_{a}\sum_{s}P(s'∣s,a)[r(s,a)+γV(s')]$$
## Why It Works
The algorithm works because of the **Markov Property**, which states that the future is independent of the past given the present. This allows the algorithm to break down the decision process into simple, manageable steps without losing the essence of the overall problem.

Value iteration is an example of **Dynamic Programming** and is guaranteed to converge to the optimal values as it effectively navigates the trade-off between exploration (trying out new actions) and exploitation (choosing the best-known action).

## Complexity 
$O(S^2*A)$ 