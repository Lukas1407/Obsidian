> [!summary] Definition
> Backpropagation computes the gradient of a loss function with respect to the weights of the network. 

> [!danger] Important
> Strictly the term _backpropagation_ refers only to the algorithm for computing the gradient, not how the gradient is used! This is for example done through [[Gradient Descent]]. However the term is often used loosely to refer to the entire learning algorithm.

The overall network is a combination of function composition and matrix multiplication:$$g(x)=f^{N}(W^{N}f^{N-1}(W^{N-1}...f^{1}(W^{1}x)...)),$$ with:
- $x$: Input
- $N$: The number of layers
- $f^{i}$: The [[Activation Function]] of the $i$-th layer
- $W^{i}$: The weight matrix of the $i$-th layer

During training, we first calculate the error for a training sample $(x_{i},y_{i})$ and our prediction $g(x_{i})$: $$L(y_{i}, g(x_{i})).$$
Backpropagation calculates the gradient of the loss with respect to each weight $w_{jk}^{i}$:$$
\frac{\partial  L}{\partial w_{jk}^{i}}
,$$but doing this separately for each weight is inefficient.
Backpropagation efficiently computes the gradient by avoiding duplicate calculations and not computing unnecessary intermediate values, by computing the gradient of each layer – specifically the gradient of the weighted _input_ of each layer, denoted by $\delta^{i}$ – from back to front.