> [!tldr] Definition
> Stochastic gradient descent (SGD) is an iterative method for optimizing an objective function with suitable smoothness properties (e.g. differentiable or subdifferentiable). It can be regarded as a stochastic approximation of [[Gradient Descent]] optimization, since it replaces the actual gradient (calculated from the entire data set) by an estimate thereof (calculated from a randomly selected subset of the data). 
This way achieving faster iterations in exchange for a lower convergence rate.

In SGD, the true gradient of $Q(w)$ is approximated by a gradient at a single sample:$$w :=w-\eta \Delta Q_{i}(w)$$
In pseudo code, stochastic gradient descent can be presented as :
```
choose an initial vector of weigts w and learning rate r
Repeat until an approximate minimum is reached:
	Randomly shuffle samples in the training set
	For i=1,...,n do:
		w = w - r * gradient(Q_i(w))
```

## Extensions and Variants
### Momentum method
A technique that adds a fraction of the previous update to the current update, to prevent oscillations and speed up convergence.$$w:=w-\eta \Delta Q_{i}(w) +\alpha \Delta w$$
- This speeds up the convergence, as it accelerates the updates if they point in the same direction
### Nesterov accelerated gradient
A modification of the momentum method that uses the gradient predicted at the next point, rather than the current point.
### Averaged SGD
A method that records an average of the parameter vector over time, and uses it as the final estimate.
### AdaGrad
AdaGrad is an optimization algorithm that adapts the learning rate for each parameter, based on the historical sum of squared gradients. 
- This means that parameters with large gradients will have a smaller learning rate, and parameters with small gradients will have a larger learning rate. 
- This can help prevent overshooting and slow down convergence in sparse features. 
- The update rule for AdaGrad is:$$

θ_t+1​=θ_{t} - \frac{\eta}{\sqrt{G_{t}+\epsilon}}⋅g_t​$$
where $θ_t$​ is the parameter vector at time step $t$, $η$ is the initial learning rate, $G_t$​ is the diagonal matrix of the sum of squared gradients up to time step $t$, $ϵ$ is a small constant to avoid division by zero, and gt​ is the gradient at time step $t$.
### RMSProp
RMSProp is an extension of AdaGrad that deals with its radically diminishing learning rates. 
- It uses a running average of squared gradients instead of a sum, which decays the influence of past gradients and allows for a more balanced learning rate. 
- The update rule for RMSProp is:$$\theta_{t+1} = \theta_t - \frac{\eta}{\sqrt{E[g^2]_t + \epsilon}} \cdot g_t
$$
where $θ_t$​ is the parameter vector at time step $t$, $η$ is the initial learning rate, $E[g2]_{t}​$ is the exponential moving average of squared gradients at time step $t$, $ϵ$ is a small constant to avoid division by zero, and $g_t$​ is the gradient at time step $t$.

### Adam
Adam is a combination of RMSProp and momentum. 
- It uses both the running average of squared gradients and the running average of gradients to adjust the learning rate and the direction of the update. 
- It also adds bias-correction terms to account for the initial values of the averages being zero. 
- The update rule for Adam is:$$\begin{aligned}
m_t &= \beta_1 m_{t-1} + (1 - \beta_1) g_t \\
v_t &= \beta_2 v_{t-1} + (1 - \beta_2) g_t^2 \\
\hat{m}_t &= \frac{m_t}{1 - \beta_1^t} \\
\hat{v}_t &= \frac{v_t}{1 - \beta_2^t} \\
\theta_{t+1} &= \theta_t - \frac{\eta}{\sqrt{\hat{v}_t + \epsilon}} \cdot \hat{m}_t
\end{aligned}
$$
where $θ_t$​ is the parameter vector at time step $t,$ $η$ is the initial learning rate, $m_t$​ and $v_t$​ are the first and second moment estimates of the gradient, $m^t​$ and $v^t​$ are the bias-corrected estimates, $β_1$​ and $β_2$​ are the decay rates for the moment estimates, $ϵ$ is a small constant to avoid division by zero, and $g_t$​ is the gradient at time step $t$.
### Sign-based SGD
A method that simplifies Adam by only considering the sign of the gradient, rather than its magnitude.
### Backtracking line search
A method that adjusts the learning rate based on a condition that ensures sufficient decrease in the objective function.
### Second-order methods
Methods that use information from the Hessian matrix or its approximations, such as the Fisher information matrix, to improve the convergence rate of SGD.