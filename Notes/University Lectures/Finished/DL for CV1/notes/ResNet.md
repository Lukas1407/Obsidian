- Can have different depths 
	- ResNet-50 has 50 layers
	- ResNet-101 has 101 layers
- Can be very deep due to [[ResNet#Residual Connection|residual connections]]

## Residual Connection
- Addresses the [[Vanishing Gradient]] and [[Exploding Gradient]] problem that occur when training deep networks
- As networks become deeper, it becomes increasingly <mark style="background: #FFB86CA6;">difficult for the model to learn the identity function</mark>, which is crucial for preserving information across layers.
- In traditional neural networks, each layer tries to learn the desired underlying mapping directly. However, as the depth increases, this becomes a challenging optimization problem due to the gradients becoming too small or too large, which hampers the learning process.
- Residual learning simplifies this by introducing the concept of a residual block. Instead of learning the direct mapping of $H(x)$, the layers learn the residual function $F(x) := H(x) - x$. The output of the residual block is then $y = F(x) + x$, where $x$ is the input to the residual block.
- The key advantage of this approach is that i<mark style="background: #FFB86CA6;">t is easier to learn the residual mapping</mark> $F(x)$ than to learn the original unreferenced mapping directly. 
	- This is because <mark style="background: #FFB86CA6;">the residual mapping can be closer to zero</mark>, especially when the underlying mapping is close to the identity function. 
	- In other words, if the optimal function is close to the identity, the network only needs to learn small deviations from it, which is a simpler task.
- The <mark style="background: #FFB86CA6;">skip connections, or shortcut connections</mark>, in ResNet allow the input $x$ to bypass one or more layers and be added to the output of a later layer. This effectively creates an identity mapping alongside the deeper layer transformations. If the additional layers do not improve the model, they can easily learn to approximate an identity function, thus not harming the performance and allowing the training to proceed effectively.

![[Pasted image 20240306091713.png#invert|]]
