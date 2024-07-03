The incremental update rule is a method used in reinforcement learning to update the estimated value of an action as new data (rewards) are observed. It’s necessary because it allows for efficient computation without the need to store all past rewards, which can be impractical, especially in long-running or continuous tasks.

- **Initial Equation**: The average of all rewards up to the $n$-th reward is given by:$$q_{n}=\frac{1}{n-1}\sum_{i=1}^{n-1}r_{i}$$
- **Incremental Update**: Instead of recalculating the entire sum each time, we can update the average incrementally using the following formula:$$\begin{align*}
q_{n} &= \frac{1}{n-1}\sum_{i=1}^{n}r_{i} \\
&= \frac{1}{n-1}(r_{n-1}+\sum_{i=1}^{n-2}r_{i}) \\
&= \frac{1}{n-1}(r_{n-1}+(n-2)\frac{1}{n-2}\sum_{i=1}^{n-2}r_{i}) \\
&= \frac{1}{n-1}(r_{n-1}+(n-2)q_{n-1}) \\
&= \frac{1}{n-1}(r_{n-1}+(n-1)q_{n-1}-q_{n-1}) \\
&= q_{n-1}+\frac{1}{n-1}(r_{n-1}-q_{n-1})
\end{align*}$$
	$$\rightarrow NewEstimate = OldEstimate + StepSize\ *\ (Target -OldEstimate)$$
	- This equation updates the estimated action-value $q_n$ to $q_{n+1}$ by adjusting it with the difference between the new reward $r_n$ and the current estimate $q_n$, scaled by $\frac{1}{n}$.
	-  This scaling factor decreases as more rewards are observed, reflecting the decreasing impact of each new reward on the overall average as the number of observations grows.
	- The incremental update rule is particularly useful because it requires constant memory and constant per-time-step computation, regardless of the number of rewards observed, making it scalable and efficient for practical applications.

- **General update rule**: $$q_{n+1}=q_{n}*\alpha_{n}*(r_{n}-q_{n})$$, with $\alpha_{n}=\frac{1}{n}$ for the sample average method
