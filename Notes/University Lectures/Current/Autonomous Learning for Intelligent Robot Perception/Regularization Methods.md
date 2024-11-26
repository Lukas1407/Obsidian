- We observe that complex models have large parameters -> leading to oscillation
- Solution: Also account for the magnitude of weights in the loss function: $$E(w)=\frac{1}{2}\sum_{i=1}^{N}(w^{T}\phi(x_{i})-y_{i})^{2}+\textcolor{orange}{\frac{\lambda}{2}||w||^{2}}$$
- Where $\lambda$ influences the weight the regularization has on the loss function
![[Pasted image 20241122123035.png#invert|400]]
- This can be written in vector form as: 
	- $$
E(w) = \frac{1}{2} w^T \Phi^T \Phi w - y^T \Phi w + \frac{1}{2} y^T y + \frac{\lambda}{2} w^T w
$$
	-  Easier than with the sum