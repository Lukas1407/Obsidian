> [!tldr] Definition
> Natural gradients are a method used in reinforcement learning to adjust the policy parameters in a way that takes into account the underlying probability distribution of the policy. This approach is more efficient than standard gradients because it scales the gradient by the inverse of the Fisher Information Matrix (FIM), which leads to faster convergence. 

- Natural Gradients use a Taylor approximation of the trust region problem, with a first-order Taylor for the objective and a second-order Taylor for the constraint: 
	- First-order Taylor: $$g_{NG}=\arg\max_{g}g^{T}\nabla_{\theta}J$$
	- Second-order Taylor: $$KL(p_{\theta_{old}+g}\parallel p_{\theta_{old}})\approx g^{T}\mathcal{F}g\le\epsilon$$, with $\mathcal{F}$ being the Fisher Information Matrix (FIM): $$\mathcal{F}=\frac{\partial KL(p_\theta\parallel p_{\theta_{old}})}{\partial\theta\partial\theta}$$The FIM is used to scale the ‘vanilla’ gradient.

......... Equations from lecture 6 slides 21-25