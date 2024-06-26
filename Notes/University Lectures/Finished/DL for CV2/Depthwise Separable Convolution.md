- Special case of grouped convolution with $g=D_{in}$ 
- Every filter group only filters 1 channel of the input volume. 
- This is very cheap computationally and has very few parameters. 
	- Depthwise: $h \times w \times D_{in} \times H_{out} \times W_{out}$
	- Pointwise: $D_{in} \times H_{out} \times W_{out} \times D_{out}$
- Depthwise separable convolution: depthwise convolution followed by a $1\times 1$ convolution ($1\times 1$ convolution is also also referred to as pointwise convolution)

![[Pasted image 20240306091834.png#invert|]]