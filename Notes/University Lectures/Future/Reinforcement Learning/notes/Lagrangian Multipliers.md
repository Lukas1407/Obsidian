> [!abstract] Definition
> Lagrangian multipliers is a method to find the local maxima and minima of a function subject to equality constraints 

**Intuition:** The basic idea behind using Lagrangian multipliers is to convert a constrained optimization problem into an unconstrained one by incorporating the constraints into the objective function with the help of additional variables called Lagrange multipliers.

**The Method:** Consider a function ( f(x, y) ) that you want to optimize subject to a constraint $g(x) = 0$. The Lagrangian $\mathcal{L}$ is defined as:$$\mathcal{L}(x,\lambda)=f(x)-\lambda*(g(x)-0)$$

## Steps to solve
1. Write down Lagrangian $$\mathcal{L}(x,\lambda)=f(x)-\lambda*(g(x)-0)$$
2. Obtain the optimal solution for the primal parameters $$\frac{\partial \mathcal{L}(x,\lambda)}{\partial x}=0 \rightarrow x^{*}=u(\lambda)$$
3. Set $x^*$ back into Lagrangian to obtain the dual function $$g(\lambda)=\mathcal{L}(u(\lambda),\lambda)$$
4. Obtain optimal solution for the dual function $$\lambda^{*}=\arg\max_{\lambda}g(\lambda), \ s.t.\ \lambda_{i}\ge 0 \ \forall i$$
5. Compute optimal primal parameters for given $\lambda^*$ $$x^{*}=u(\lambda^{*})$$
## Examples
### $\min_{x}x^{2} \ s.t. \ x\ge 1$
1. $$\mathcal{L}(x,\lambda)=x^{2}-\lambda*(x-1)$$
2. $$\frac{\partial \mathcal{L}(x,\lambda)}{\partial x}=0 \rightarrow 2x-\lambda=0\rightarrow x^{*}=\frac{\lambda}{2}=u(\lambda)$$
3. $$g(\lambda)=\left(\frac{\lambda}{2}\right)^{2}-\lambda*(\frac{\lambda}{2}-1)=\frac{\lambda^{2}}{4}-\frac{\lambda^{2}}{2}-\lambda=\frac{-\lambda^{2}}{4}+\lambda$$
4. $$\frac{-2\lambda}{4}+1 =0 \rightarrow \lambda^{*}=2$$
5. $$x^{*}=\frac{2}{2}=1$$