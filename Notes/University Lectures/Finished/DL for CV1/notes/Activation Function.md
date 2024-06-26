- Must be nonlinear, as otherwise the model would collapse to a linear decision surface
- -> Only few and easy problems would be decidable (XOR wouldn't)

> [!note] 
> When the activation function is non-linear, then a two-layer neural network can be proven to be a universal function approximator [G. Cybenko](https://hal.science/hal-03753170/file/Cybenko1989.pdf). This is known as the Universal Approximation Theorem.

## Sigmoid
$$\sigma=\frac{1}{1+e^{-x}}=1-\sigma(-x)$$
- ![[Pasted image 20240304075715.png#invert|300]]
- Large positive input -> 1
- Large negative input -> 0
- <mark style="background: #BBFABBA6;">Advantages</mark>:
	- Easy to calculate the derivative:$$\sigma'=\sigma(1-\sigma)$$
	- Clear interpretation as a probability or a binary output
	- It is normalized, which means it can handle inputs of different scales and ranges without exploding for example
- <mark style="background: #FF5582A6;">Disadvantages</mark>:
	- Suffers from [[Vanishing Gradient]]: if the inputs are too large or too small, the gradient becomes almost 0, leading to no/little learning
	- Not zero-centered: 
		- It can <mark style="background: #FF5582A6;">introduce a bias shift in the activations</mark> of the neurons, which can affect how the network learns and generalizes
		- It can make the gradient updates for the weights more difficult, because they will always have the same sign and direction, leading to <mark style="background: #FF5582A6;">zig-zagging dynamics</mark> or slow convergence.

## Hyperbolic Tangent (tanh)
$$tanh(x)=\frac{e^{x}-e^{-x}}{e^{x}+e^{-x}}$$
- ![[Pasted image 20240304081047.png#invert|300]]
- Similar to sigmoid but zero-centered
- Large positive input -> 1
- Large negative input -> -1
- <mark style="background: #BBFABBA6;">Advantages</mark>:
	- Easy to calculate the derivative:$$tanh(x)'=1-tanh^2(x)$$
	- It is normalized, which means it can handle inputs of different scales and ranges without exploding for example
	- Zero-centered: 
		- It can reduce the bias shift in the activations of the neurons, which can improve the network’s performance and generalization
		- It can make the gradient updates for the weights more efficient, because they will have different signs and directions, leading to faster convergence 
- <mark style="background: #FF5582A6;">Disadvantages</mark>:
	- Suffers from [[Vanishing Gradient]]: if the inputs are too large or too small, the gradient becomes almost 0, leading to no/little learning

## Rectified Linear Unit (ReLU)
$$ReLU(x)=x^{+}=max(0,x)=\left\{\begin{array}{ll} x, & if\ x>0 \\
         0, & otherwise\end{array}\right. $$
![[Pasted image 20240304081750.png#invert|300]]
- Advantages:
	- Sparse activation: For example, in a randomly initialized network, only about 50% of hidden units are activated (have a non-zero output)
	- Better gradient propagation: Fewer vanishing gradient problems compared to sigmoidal activation functions that saturate in both directions
	- Efficient computation: Only comparison, addition and multiplication.
- Disadvantage:
	- Non-differentiable at zero; however, it is differentiable anywhere else, and the value of the derivative at zero can be arbitrarily chosen to be 0 or 1
	- Not zero-centered
	- Unbounded
	- Dying ReLU problem: Large gradient flows to a neuron can cause it to update in such a way that the neuron will never activate again (weights get updated to be highly negative) -> The gradient from that neuron will forever be 0
### Leaky ReLU
$$LeakyReLU(x)=\left\{\begin{array}{ll} x, & if\ x>0 \\
         ax, & otherwise\end{array}\right. $$
- Has a small gradient of $a$ instead of 0
- This helps mitigate the dying ReLU problem
### Gaussian-Error Linear Unit (GELU)
$$GELU(x)=x * \Phi(x),$$where $\Phi (x)$ is the standard Gaussian cumulative distribution function.
- The GELU nonlinearity weights inputs by their percentile, rather than gates inputs by their sign as in [[Activation Function#Rectified Linear Unit (ReLU)|ReLU]]
- -> GELU can be thought of as a smoother ReLU