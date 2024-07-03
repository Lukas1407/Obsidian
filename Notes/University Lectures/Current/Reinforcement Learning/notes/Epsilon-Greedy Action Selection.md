The main idea is to exploit the knowledge with probability of $1-\epsilon$ and explore with probability of $\epsilon$
$$
\begin{align*}
a_{t}\leftarrow 
\left\{
    \begin {aligned}
         & \operatorname*{argmax} \limits_{x}q_{t}(a)  \quad & with\ probability\ 1-\epsilon \\
         & a \sim Uniform(a_{1},...,a_{k}) \quad & with\ probability\ \epsilon                  
    \end{aligned}
\right.
\end{align*}
$$

## Results
![[Pasted image 20240310100726.png#invert|]]
- The curve representing the performance of an epsilon-greedy policy can be noisy due to the randomness introduced by exploration. Even when a good policy is being exploited, occasional exploration (choosing a random action) can result in receiving lower rewards, thus creating fluctuations in the performance curve.
- Higher values of epsilon increase the amount of exploration, which can lead to discovering better policies. However, too much exploration can also mean that the agent spends less time exploiting the best-known actions, which can slow down the learning process and lead to a lower overall reward.
- If epsilon is set too high, the agent will mostly explore, choosing random actions most of the time. This can prevent the agent from sufficiently exploiting the best-known actions to accumulate a higher reward. Essentially, the agent would be behaving more randomly, which is not conducive to learning a good policy.
- -> The ideal epsilon value often starts higher for initial exploration and is gradually decreased over time to allow more exploitation of the best-known actions as the agent learns.